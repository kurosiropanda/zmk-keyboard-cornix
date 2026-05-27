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
 * rmk-firmware/: a HaoboGu RMK build with no charge-status GPIO — pinmux.c
 * drives P0.05 as a charger *control* output, battery is read only via SAADC).
 * So, like the stock firmware, "charging" is inferred from VBUS power and
 * "complete" from the battery gauge reaching CONFIG_CORNIX_STATUS_LED_BATTERY_FULL
 * while powered. VBUS is read via vbus_present(): the central uses
 * zmk_usb_is_powered(); the peripheral has CONFIG_ZMK_USB off (enabling it slows
 * input and reports VBUS=false anyway) so it reads the nRF USBREGSTATUS
 * VBUS-detect bit directly — no USB stack needed. Both halves show charging.
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
#else
/* No ZMK USB stack (e.g. the peripheral, where enabling it slows input): read
 * the nRF USB regulator's VBUS-detect bit directly — works with no USB stack. */
#include <hal/nrf_power.h>
#endif

/* True while 5V VBUS is present on this half's USB port (i.e. charging). */
static inline bool vbus_present(void) {
#if HAS_USB
    return zmk_usb_is_powered();
#else
    return nrf_power_usbregstatus_vbusdet_get(NRF_POWER);
#endif
}

LOG_MODULE_REGISTER(cornix_status_led, CONFIG_ZMK_LOG_LEVEL);

#define STRIP_NODE DT_ALIAS(status_ws2812)
#define STRIP_NPIX DT_PROP(STRIP_NODE, chain_length)

static const struct device *const strip = DEVICE_DT_GET(STRIP_NODE);

/* ---- LED roles (chain index). Swap if physical mapping differs. ---- */
#if IS_CENTRAL
/* Verified on hardware via the diagnostic build: chain index 0 = inner LED,
 * index 1 = outer LED. Manual maps the left half's outer (左側) light to
 * Bluetooth and the inner (右側) light to unit-link + battery. */
#define LED_BT 1   /* outer (左側ライト): Bluetooth */
#define LED_AUX 0  /* inner (右側ライト): right-unit link + battery/charge */
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
static const struct led_rgb COL_GREEN = {.r = 0, .g = SCALE(255), .b = 0};

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
 * Read Zephyr's live connection table directly instead of the
 * zmk_ble_active_profile_* / zmk_split_peripheral_status APIs, which key off
 * bonded/profile addresses and proved unreliable here (the BT API returns
 * false for RPA hosts; the split status event didn't update the cache) — so
 * both indicator LEDs stayed stuck blinking. On the split central the keyboard
 * is the PERIPHERAL on the host link and the CENTRAL on the split link, so:
 *   host connected      = any connected PERIPHERAL-role LE connection
 *   right unit connected = any connected CENTRAL-role LE connection
 */
struct role_count {
    uint8_t role;
    int n;
};
static void role_count_cb(struct bt_conn *conn, void *data) {
    struct role_count *rc = data;
    struct bt_conn_info info;
    if (bt_conn_get_info(conn, &info) == 0 && info.role == rc->role &&
        info.state == BT_CONN_STATE_CONNECTED) {
        rc->n++;
    }
}
static bool any_conn(uint8_t role) {
    struct role_count rc = {.role = role, .n = 0};
    bt_conn_foreach(BT_CONN_TYPE_LE, role_count_cb, &rc);
    return rc.n > 0;
}
#define host_connected() any_conn(BT_CONN_ROLE_PERIPHERAL)
#define split_unit_connected() any_conn(BT_CONN_ROLE_CENTRAL)
#endif

/* ---- Patterns. Ongoing states "breathe" (smooth fade up/down) over one
 * period; "一度点灯後消灯" stays a single solid pulse. ---- */
#define SLOW_PERIOD_MS 1800 /* ゆっくり: searching / link lost / charging */
#define FAST_PERIOD_MS 700  /* 点滅(速め): low battery */
#define ONESHOT_MS 1000     /* 一度点灯後消灯: connection / charge complete */
#define FRAME_MS 50         /* animation step (thread only ticks while animating) */
#define IDLE_POLL_MS 2000   /* re-poll link/VBUS when idle (some inputs aren't pushed) */

/* Raised-cosine breathing curve, 0->255->0 over 32 steps (perceptually smooth
 * ease at both ends). Indexed by the phase position within the period. */
__maybe_unused static const uint8_t breath_lut[32] = {
    0,   2,   10,  21,  37,  57,  79,  103, 128, 152, 176, 198, 218, 234, 245, 253,
    255, 253, 245, 234, 218, 198, 176, 152, 128, 103, 79,  57,  37,  21,  10,  2,
};

/*
 * TEMPORARY diagnostic mode. When 1, both LEDs show SOLID colours encoding the
 * raw internal signals (no blinking), so we can see ground truth on hardware:
 *   Central (left):  LED0 = host_connected()? GREEN:RED   (BT host link)
 *                    LED1 = split connected?  BLUE:MAGENTA (right-unit link)
 *   Peripheral(right): LED0 = central link?   GREEN:RED
 *                      LED1 = always BLUE (position marker)
 * Colour families (green/red vs blue/magenta) also reveal the chain index ->
 * physical-side mapping. Set back to 0 for normal operation.
 */
#define CORNIX_LED_DIAG 0

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
static bool s_batt_low;     /* SoC <= LOW threshold (from battery events) */
static bool s_batt_full;    /* SoC >= FULL threshold (from battery events) */
static bool s_chg_done;     /* edge latch for the charge-complete one-shot */
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

/* Charge complete: one green pulse when SoC reaches full while on VBUS power. */
static void note_charge(bool powered) {
    bool full_charge = powered && s_batt_full;
    if (full_charge && !s_chg_done) {
        fire_oneshot(LED_BATT, COL_GREEN); /* 充電完了: 一度点灯後消灯 */
    }
    s_chg_done = full_charge;
}

/* Derive each LED's ongoing pattern from the tracked state (manual mapping). */
__maybe_unused static void recompute_modes(void) {
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

    /* Right unit (split) link — also read live; the cached status event left
     * this stuck "disconnected", blue-blinking the inner LED forever. */
    bool periph = split_unit_connected();
    if (periph && !s_periph_connected) {
        fire_oneshot(LED_AUX, COL_BLUE); /* right unit connected -> light once then off */
    }
    s_periph_connected = periph;

    /* Right light: right-unit link / charge / battery (one light, by priority). */
    bool powered = vbus_present();
    note_charge(powered);
    if (!s_periph_connected) {
        set_mode(LED_AUX, MODE_SLOW, COL_BLUE);   /* right unit lost */
    } else if (powered && !s_batt_full) {
        set_mode(LED_AUX, MODE_SLOW, COL_GREEN);  /* charging */
    } else if (s_batt_low && !powered) {
        set_mode(LED_AUX, MODE_FAST, COL_RED);    /* low battery */
    } else {
        set_mode(LED_AUX, MODE_OFF, COL_OFF);     /* idle / charge complete */
    }
#else
    /* Left light: battery / charge. */
    bool powered = vbus_present();
    note_charge(powered);
    if (powered && !s_batt_full) {
        set_mode(LED_BATT, MODE_SLOW, COL_GREEN); /* charging */
    } else if (s_batt_low && !powered) {
        set_mode(LED_BATT, MODE_FAST, COL_RED);   /* low battery */
    } else {
        set_mode(LED_BATT, MODE_OFF, COL_OFF);    /* idle / charge complete */
    }

    /* Right light: left-unit link. */
    set_mode(LED_UNIT, s_unit_connected ? MODE_OFF : MODE_SLOW, COL_BLUE);
#endif
}

__maybe_unused static struct led_rgb scale_rgb(struct led_rgb c, uint8_t lvl) {
    struct led_rgb o = {
        .r = (uint8_t)((uint16_t)c.r * lvl / 255),
        .g = (uint8_t)((uint16_t)c.g * lvl / 255),
        .b = (uint8_t)((uint16_t)c.b * lvl / 255),
    };
    return o;
}

/* Colour this LED should show right now; sets *active if it still animates. */
__maybe_unused static struct led_rgb render_led(const struct led_state *st, int64_t now,
                                                bool *active) {
    if (now < st->oneshot_until) {
        *active = true;
        return st->oneshot_color; /* "点灯後消灯": solid pulse, no breathing */
    }
    if (st->mode != MODE_OFF) {
        *active = true;
        uint32_t period = (st->mode == MODE_SLOW) ? SLOW_PERIOD_MS : FAST_PERIOD_MS;
        uint32_t idx = (uint32_t)((now % period) * 32 / period) & 31u;
        return scale_rgb(st->color, breath_lut[idx]); /* breathe up/down */
    }
    return COL_OFF;
}

/* Render one frame; returns true while something still needs animating. */
static bool render_frame(void) {
#if CORNIX_LED_DIAG
    const struct led_rgb GRN = {.r = 0, .g = SCALE(255), .b = 0};
    const struct led_rgb RED = {.r = SCALE(255), .g = 0, .b = 0};
    const struct led_rgb BLU = {.r = 0, .g = 0, .b = SCALE(255)};
    const struct led_rgb MAG = {.r = SCALE(255), .g = 0, .b = SCALE(255)};
    struct led_rgb d[2];
#if IS_CENTRAL
    d[0] = host_connected() ? GRN : RED;
    d[1] = s_periph_connected ? BLU : MAG;
#else
    d[0] = vbus_present() ? GRN : RED;       /* VBUS present (charging)? */
    d[1] = s_unit_connected ? BLU : MAG;     /* split link to central */
#endif
    bool ch = false;
    for (size_t i = 0; i < STRIP_NPIX && i < 2; i++) {
        if (!rgb_eq(d[i], shown[i])) {
            shown[i] = d[i];
            ch = true;
        }
    }
    if (ch && device_is_ready(strip)) {
        led_strip_update_rgb(strip, shown, STRIP_NPIX);
    }
    return true; /* keep polling so signal changes show up */
#else
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
#endif /* CORNIX_LED_DIAG */
}

static void led_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    while (1) {
        bool active = render_frame();
        /*
         * Animating -> tick at FRAME_MS; idle -> re-poll every IDLE_POLL_MS.
         * We poll even when idle because the inputs are read live, not pushed:
         * the BLE link state (RPA hosts miss the profile event) and VBUS/charging
         * (read from the nRF register, no event on the peripheral). A state-change
         * event (kick) still wakes us instantly for responsiveness.
         */
        k_sem_take(&wake_sem, active ? K_MSEC(FRAME_MS) : K_MSEC(IDLE_POLL_MS));
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
        s_batt_full = batt->state_of_charge >= CONFIG_CORNIX_STATUS_LED_BATTERY_FULL;
        /* recompute_modes() reads VBUS live and fires the charge-complete pulse. */
        kick();
        return ZMK_EV_EVENT_BUBBLE;
    }

#if HAS_USB
    if (as_zmk_usb_conn_state_changed(eh)) {
        kick(); /* VBUS change; recompute_modes() reads it live via vbus_present() */
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
    if (as_zmk_split_peripheral_status_changed(eh)) {
        /* recompute_modes() reads the live split link; just wake the thread. */
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

    /* Seed state so boot-time conditions render correctly (VBUS/charge are
     * read live in recompute_modes; nothing to seed here for them). */
#if IS_CENTRAL
    s_bt_index = zmk_ble_active_profile_index();
    s_bt_connected = host_connected();
    s_periph_connected = split_unit_connected();
#endif
    kick(); /* render the seeded state once the thread starts */
    return 0;
}

SYS_INIT(cornix_status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
