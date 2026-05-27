/*
 * Cornix WS2812 status indicator (manual-style scheme)
 *
 * Drives the 2 WS2812 LEDs (devicetree alias `status-ws2812`, chain-length 2)
 * to reproduce the stock-firmware indicator behaviour documented in
 * doc/cornix-manual-ja.md.
 *
 * Manual spec -> behaviour
 * ------------------------
 * Left half (central):
 *   LED0 (left light, Bluetooth):
 *     - channel colour ch0 green / ch1 red / ch2 blue
 *     - searching (not connected) -> slow blink in channel colour
 *     - connected                 -> light once then off
 *   LED1 (right light, right-unit link + battery/charge):
 *     - right unit lost            -> blue slow blink
 *     - right unit connected       -> blue light once then off
 *     - charging                   -> green slow blink
 *     - charge complete            -> green light once then off
 *     - low battery (on battery)   -> red blink (faster)
 * Right half (peripheral):
 *   LED0 (left light, battery/charge): charging / complete / low battery
 *     (same colours/patterns as above)
 *   LED1 (right light, left-unit link): lost -> blue slow blink,
 *     connected -> blue light once then off
 *
 * Pattern timing (the manual distinguishes "点滅"/blink from "ゆっくり点滅"/slow
 * blink): slow blink = SLOW_ON/SLOW_PERIOD, blink = FAST_ON/FAST_PERIOD,
 * "light once then off" = one solid ONESHOT_MS pulse.
 *
 * Charge detection (confirmed by analysing the stock RMK firmware in
 * rmk-firmware/: it is a HaoboGu RMK build with no charge-status GPIO —
 * pinmux.c drives P0.05 as a charger *control* output, battery is read only
 * via SAADC). So, like the stock firmware, "charging" is inferred from USB
 * VBUS power (zmk_usb_is_powered) and "complete" from the battery gauge
 * reaching CONFIG_CORNIX_STATUS_LED_BATTERY_FULL while powered. Needs
 * CONFIG_ZMK_USB; the right half disables USB by default, so its charge
 * state is compiled out unless USB is enabled in the shield .conf.
 *
 * Threading/latency: ALL rendering runs on a dedicated low-priority thread,
 * never the system work queue. ZMK raises keyboard input, HID and even the
 * BLE profile-changed event via the system work queue, so blocking it with
 * SPI strip updates would both lag key input and drop connection events
 * (leaving the indicator stuck "searching"). The thread blocks on a
 * semaphore while idle and only animates while a blink is active.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/split_peripheral_status_changed.h>

#define IS_CENTRAL (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))
#define HAS_USB IS_ENABLED(CONFIG_ZMK_USB)

#if IS_CENTRAL
#include <zephyr/bluetooth/conn.h>
#include <zmk/ble.h>
#include <zmk/events/ble_active_profile_changed.h>
#else
#include <zmk/split/bluetooth/peripheral.h>
#endif

#if HAS_USB
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#endif

LOG_MODULE_REGISTER(cornix_status_led, CONFIG_ZMK_LOG_LEVEL);

#define STRIP_NODE DT_ALIAS(status_ws2812)
#define STRIP_NPIX DT_PROP(STRIP_NODE, chain_length)

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

/* ---- LED roles (chain index). Swap if physical mapping differs. ---- */
#if IS_CENTRAL
#define LED_BT 0   /* left light  : Bluetooth */
#define LED_AUX 1  /* right light : right-unit link + battery/charge */
#define LED_BATT LED_AUX
#else
#define LED_BATT 0 /* left light  : battery/charge */
#define LED_UNIT 1 /* right light : left-unit link */
#endif

/* ---- Colours (brightness-scaled) ---- */
#define B CONFIG_CORNIX_STATUS_LED_BRIGHTNESS
#define SCALE(x) ((uint8_t)((x) * (B) / 255))
static const struct led_rgb COL_RED = {.r = SCALE(255), .g = 0, .b = 0};
static const struct led_rgb COL_BLUE = {.r = 0, .g = 0, .b = SCALE(255)};
static const struct led_rgb COL_OFF = {.r = 0, .g = 0, .b = 0};
/* Green is only used by the BT channel colour (central) or charging (USB);
 * guard it so the USB-less right half doesn't trip -Wunused-const-variable. */
#if IS_CENTRAL || HAS_USB
static const struct led_rgb COL_GREEN = {.r = 0, .g = SCALE(255), .b = 0};
#endif

#if IS_CENTRAL
static struct led_rgb chan_color(int idx) {
    switch (idx) {
    case 0:
        return COL_GREEN;
    case 1:
        return COL_RED;
    default:
        return COL_BLUE;
    }
}

/*
 * Is a host (PC/phone) connected over BLE? On the split central the keyboard
 * is the PERIPHERAL on host links and the CENTRAL on the split link, so a
 * connected peripheral-role LE connection means "host connected". This reads
 * Zephyr's live connection table directly, bypassing zmk_ble_active_profile_*
 * which keys off the bonded profile address and returns false for hosts using
 * resolvable private addresses (RPA) — the reason the BT LED stayed blinking.
 */
static void host_conn_count_cb(struct bt_conn *conn, void *data) {
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0 && info.role == BT_CONN_ROLE_PERIPHERAL &&
        info.state == BT_CONN_STATE_CONNECTED) {
        (*(int *)data)++;
    }
}

static bool host_connected(void) {
    int n = 0;
    bt_conn_foreach(BT_CONN_TYPE_LE, host_conn_count_cb, &n);
    return n > 0;
}
#endif

/* ---- Blink patterns: "ゆっくり点滅" vs "点滅" vs "点灯後消灯" ---- */
#define SLOW_ON_MS 600     /* ゆっくり点滅: searching / link lost / charging */
#define SLOW_PERIOD_MS 1200
#define FAST_ON_MS 250     /* 点滅: low battery */
#define FAST_PERIOD_MS 500
#define ONESHOT_MS 1000    /* 一度点灯後消灯: connection / charge complete */
#define FRAME_MS 50        /* animation step (thread only ticks while animating) */
#define IDLE_POLL_MS 2000  /* central: re-poll BT link when idle (missed events) */

enum led_mode { MODE_OFF, MODE_SLOW, MODE_FAST };

struct led_state {
    enum led_mode mode;          /* ongoing pattern */
    struct led_rgb color;        /* colour for the ongoing pattern */
    int64_t oneshot_until;       /* if > now, show oneshot_color solid */
    struct led_rgb oneshot_color;
};

static struct led_state leds[STRIP_NPIX];
static struct led_rgb shown[STRIP_NPIX]; /* what's currently on the strip */

/* ---- Tracked state (set by events, read by the render thread) ---- */
static bool s_batt_low;
#if HAS_USB
static bool s_powered;
static bool s_batt_full;
#endif
#if IS_CENTRAL
static int s_bt_index;
static bool s_bt_connected;
static bool s_periph_connected;
#else
static bool s_unit_connected;
#endif

/* Wakes the render thread on any state change (see threading note up top). */
K_SEM_DEFINE(wake_sem, 0, 1);

static bool rgb_eq(struct led_rgb a, struct led_rgb b) {
    return a.r == b.r && a.g == b.g && a.b == b.b;
}

static void set_mode(int led, enum led_mode mode, struct led_rgb color) {
    leds[led].mode = mode;
    leds[led].color = color;
}

/* "一度点灯後消灯": light this LED solid for ONESHOT_MS, then resume its mode. */
static void fire_oneshot(int led, struct led_rgb color) {
    leds[led].oneshot_color = color;
    leds[led].oneshot_until = k_uptime_get() + ONESHOT_MS;
}

/* Derive each LED's ongoing pattern from the tracked state (manual mapping). */
static void recompute_modes(void) {
#if IS_CENTRAL
    /*
     * Left light: Bluetooth. Use the live Zephyr connection table (see
     * host_connected) rather than zmk_ble_active_profile_is_connected(), which
     * returned false for RPA hosts and left this LED stuck blinking. The
     * channel index still picks the colour. Pulse once on the connect edge.
     */
    int bt_idx = zmk_ble_active_profile_index();
    bool bt_conn = host_connected();
    if (bt_conn && !s_bt_connected) {
        fire_oneshot(LED_BT, chan_color(bt_idx)); /* connected -> light once then off */
    }
    s_bt_connected = bt_conn;
    s_bt_index = bt_idx;
    set_mode(LED_BT, bt_conn ? MODE_OFF : MODE_SLOW, chan_color(bt_idx));

    /* Right light: right-unit link / charge / battery (one light, by priority). */
    if (!s_periph_connected) {
        set_mode(LED_AUX, MODE_SLOW, COL_BLUE);
#if HAS_USB
    } else if (s_powered) {
        set_mode(LED_AUX, s_batt_full ? MODE_OFF : MODE_SLOW, COL_GREEN);
#endif
    } else if (s_batt_low) {
        set_mode(LED_AUX, MODE_FAST, COL_RED);
    } else {
        set_mode(LED_AUX, MODE_OFF, COL_OFF);
    }
#else
    /* Left light: battery / charge. */
#if HAS_USB
    if (s_powered) {
        set_mode(LED_BATT, s_batt_full ? MODE_OFF : MODE_SLOW, COL_GREEN);
    } else if (s_batt_low) {
#else
    if (s_batt_low) {
#endif
        set_mode(LED_BATT, MODE_FAST, COL_RED);
    } else {
        set_mode(LED_BATT, MODE_OFF, COL_OFF);
    }

    /* Right light: left-unit link. */
    set_mode(LED_UNIT, s_unit_connected ? MODE_OFF : MODE_SLOW, COL_BLUE);
#endif
}

/* Colour this LED should show right now; sets *active if it still animates. */
static struct led_rgb render_led(const struct led_state *st, int64_t now, bool *active) {
    if (now < st->oneshot_until) {
        *active = true;
        return st->oneshot_color;
    }
    if (st->mode != MODE_OFF) {
        *active = true;
        uint32_t period = (st->mode == MODE_SLOW) ? SLOW_PERIOD_MS : FAST_PERIOD_MS;
        uint32_t on = (st->mode == MODE_SLOW) ? SLOW_ON_MS : FAST_ON_MS;
        uint32_t phase = (uint32_t)(now % period);
        return (phase < on) ? st->color : COL_OFF;
    }
    return COL_OFF;
}

/* Render one frame; returns true while something still needs animating. */
static bool render_frame(void) {
    recompute_modes();
    int64_t now = k_uptime_get();
    bool active = false;
    bool changed = false;

    for (size_t i = 0; i < STRIP_NPIX; i++) {
        struct led_rgb px = render_led(&leds[i], now, &active);
        if (!rgb_eq(px, shown[i])) {
            shown[i] = px;
            changed = true;
        }
    }
    if (changed && device_is_ready(strip)) {
        led_strip_update_rgb(strip, shown, STRIP_NPIX);
    }
    return active;
}

static void led_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    while (1) {
        bool active = render_frame();
        /*
         * Animating -> tick at FRAME_MS; idle -> sleep until a state change.
         * On the central we re-poll every IDLE_POLL_MS even when idle, because
         * the BLE profile-changed event can be missed (RPA hosts), so a
         * connect/disconnect must still be picked up by polling is_connected().
         * A state-change event (kick) wakes us instantly regardless.
         */
        k_timeout_t idle = IS_CENTRAL ? K_MSEC(IDLE_POLL_MS) : K_FOREVER;
        k_sem_take(&wake_sem, active ? K_MSEC(FRAME_MS) : idle);
    }
}

/* Lowest-priority preemptible thread so it never delays input; 2 KB stack
 * covers the blocking SPI strip update. Started ~1 s after boot. */
K_THREAD_DEFINE(cornix_led_tid, 2048, led_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, 1000);

static void kick(void) {
    k_sem_give(&wake_sem);
}

/* ---- Event handling (runs on the system work queue; must not block) ---- */
static int led_event_cb(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *batt = as_zmk_battery_state_changed(eh);
    if (batt) {
        s_batt_low = batt->state_of_charge <= CONFIG_CORNIX_STATUS_LED_BATTERY_LOW;
#if HAS_USB
        bool full = batt->state_of_charge >= CONFIG_CORNIX_STATUS_LED_BATTERY_FULL;
        if (s_powered && full && !s_batt_full) {
            fire_oneshot(LED_BATT, COL_GREEN); /* charge complete */
        }
        s_batt_full = full;
#endif
        kick();
        return ZMK_EV_EVENT_BUBBLE;
    }

#if HAS_USB
    const struct zmk_usb_conn_state_changed *usb = as_zmk_usb_conn_state_changed(eh);
    if (usb) {
        bool powered = zmk_usb_is_powered();
        if (powered && !s_powered && s_batt_full) {
            fire_oneshot(LED_BATT, COL_GREEN); /* plugged in already full */
        }
        s_powered = powered;
        kick();
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

#if IS_CENTRAL
    if (as_zmk_ble_active_profile_changed(eh)) {
        /* recompute_modes() reads the live BT state and pulses on connect;
         * just wake the thread (this event is unreliable for RPA hosts). */
        kick();
        return ZMK_EV_EVENT_BUBBLE;
    }
    const struct zmk_split_peripheral_status_changed *ps =
        as_zmk_split_peripheral_status_changed(eh);
    if (ps) {
        bool was = s_periph_connected;
        s_periph_connected = ps->connected;
        if (s_periph_connected && !was) {
            fire_oneshot(LED_AUX, COL_BLUE); /* right unit connected */
        }
        kick();
        return ZMK_EV_EVENT_BUBBLE;
    }
#else
    if (as_zmk_split_peripheral_status_changed(eh)) {
        bool was = s_unit_connected;
        s_unit_connected = zmk_split_bt_peripheral_is_connected();
        if (s_unit_connected && !was) {
            fire_oneshot(LED_UNIT, COL_BLUE); /* left unit connected */
        }
        kick();
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(cornix_status_led, led_event_cb);
ZMK_SUBSCRIPTION(cornix_status_led, zmk_battery_state_changed);
ZMK_SUBSCRIPTION(cornix_status_led, zmk_split_peripheral_status_changed);
#if HAS_USB
ZMK_SUBSCRIPTION(cornix_status_led, zmk_usb_conn_state_changed);
#endif
#if IS_CENTRAL
ZMK_SUBSCRIPTION(cornix_status_led, zmk_ble_active_profile_changed);
#endif

static int cornix_status_led_init(void) {
    if (!device_is_ready(strip)) {
        LOG_ERR("WS2812 strip device not ready");
        return -ENODEV;
    }
    for (size_t i = 0; i < STRIP_NPIX; i++) {
        shown[i] = COL_OFF;
    }
    led_strip_update_rgb(strip, shown, STRIP_NPIX);

    /* Seed state so boot-time conditions render correctly. */
#if HAS_USB
    s_powered = zmk_usb_is_powered();
#endif
#if IS_CENTRAL
    s_bt_index = zmk_ble_active_profile_index();
    s_bt_connected = host_connected();
#endif
    kick(); /* render the seeded state once the thread starts */
    return 0;
}

SYS_INIT(cornix_status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
