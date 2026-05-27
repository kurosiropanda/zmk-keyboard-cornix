/*
 * Cornix WS2812 status indicator (manual-style scheme)
 *
 * Drives the 2 WS2812 LEDs (devicetree alias `status-ws2812`, chain-length 2)
 * to mirror the stock-firmware indicator behaviour documented in
 * doc/cornix-manual-ja.md.
 *
 * Per-half roles (see manual):
 *   Left half  (central):  LED0 = Bluetooth (channel colour / connection),
 *                          LED1 = split link to right + own battery/charge.
 *   Right half (peripheral): LED0 = own battery/charge,
 *                            LED1 = split link to left.
 *
 * Colour scheme:
 *   - BT channel colour: ch0 = green, ch1 = red, ch2 = blue.
 *   - Connected   -> single blink (on-then-off), then LED stays off.
 *   - Searching/lost -> slow periodic blink while in that state.
 *   - Low battery -> red slow blink (only while NOT on USB power).
 *   - Charging    -> green slow blink while on USB power and not yet full.
 *   - Charge done -> single green blink when reaching full while powered.
 *
 * Charge detection (matches stock RMK, which has no charge-status pin —
 * see boards/jzf/cornix/pinmux.c: BOARD_CORNIX_CHARGER drives P0.05 as a
 * charger *control* output, not a status input): "charging" is inferred
 * from USB VBUS power (zmk_usb_is_powered), "complete" from battery SoC
 * reaching CONFIG_CORNIX_STATUS_LED_BATTERY_FULL while powered. This needs
 * CONFIG_ZMK_USB=y; the right half ships with USB disabled ("never output
 * usb"), so its charge indication is compiled out unless USB is enabled.
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
static struct led_rgb pixels[STRIP_NPIX]; /* steady state: all off */

/* ---- LED roles (chain index). Swap if physical mapping differs. ---- */
#if IS_CENTRAL
#define LED_BT 0   /* left light  : Bluetooth */
#define LED_AUX 1  /* right light : peripheral link + battery/charge */
#define LED_BATT LED_AUX
#else
#define LED_BATT 0 /* left light  : battery/charge */
#define LED_UNIT 1 /* right light : split link to central */
#endif

/* ---- Colours (brightness-scaled) ---- */
#define B CONFIG_CORNIX_STATUS_LED_BRIGHTNESS
#define SCALE(x) ((uint8_t)((x) * (B) / 255))
static const struct led_rgb COL_RED = {.r = SCALE(255), .g = 0, .b = 0};
static const struct led_rgb COL_GREEN = {.r = 0, .g = SCALE(255), .b = 0};
static const struct led_rgb COL_BLUE = {.r = 0, .g = 0, .b = SCALE(255)};
static const struct led_rgb COL_OFF = {.r = 0, .g = 0, .b = 0};

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
#endif

/* ---- Blink engine: serialise one-shot / periodic blinks via a queue ---- */
struct led_cmd {
    struct led_rgb color;
    uint8_t led;
    uint16_t on_ms;
    uint16_t off_ms;
};

K_MSGQ_DEFINE(led_q, sizeof(struct led_cmd), 8, 4);

static void strip_show(void) {
    if (device_is_ready(strip)) {
        led_strip_update_rgb(strip, pixels, STRIP_NPIX);
    }
}

static void led_thread(void *a, void *b, void *c) {
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    struct led_cmd cmd;
    while (1) {
        k_msgq_get(&led_q, &cmd, K_FOREVER);
        pixels[cmd.led] = cmd.color;
        strip_show();
        k_msleep(cmd.on_ms);
        pixels[cmd.led] = COL_OFF;
        strip_show();
        if (cmd.off_ms) {
            k_msleep(cmd.off_ms);
        }
    }
}

K_THREAD_DEFINE(cornix_led_tid, 1024, led_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(10), 0, 0);

static void blink(uint8_t led, struct led_rgb color, uint16_t on_ms, uint16_t off_ms) {
    struct led_cmd cmd = {.color = color, .led = led, .on_ms = on_ms, .off_ms = off_ms};
    k_msgq_put(&led_q, &cmd, K_NO_WAIT);
}

#define BLINK_SLOW_MS 200
#define BLINK_EVENT_MS 400

/* ---- Tracked state (updated in event callback, read in tick) ---- */
static bool s_batt_low;
#if HAS_USB
static bool s_powered;
static bool s_batt_full;
#endif
#if IS_CENTRAL
static int s_bt_index;
static bool s_bt_connected;
static bool s_bt_open;
static bool s_periph_connected;
#else
static bool s_unit_connected;
#endif

/* ---- Periodic "attention" tick: slow-blink ongoing states ---- */
static struct k_work_delayable tick_work;

/*
 * Battery/charge LED (central: LED_AUX, also shows split-link; peripheral:
 * LED_BATT). Priority while ticking: split-link loss (central) > charging >
 * low battery. Charge-complete and split-reconnect are one-shots fired from
 * the event callback, so they are not repeated here.
 */
static void tick(struct k_work *w) {
    ARG_UNUSED(w);
#if IS_CENTRAL
    /* Bluetooth: blink channel colour while searching (open & not connected). */
    if (s_bt_open && !s_bt_connected) {
        blink(LED_BT, chan_color(s_bt_index), BLINK_SLOW_MS, 0);
    }
    if (!s_periph_connected) {
        blink(LED_AUX, COL_BLUE, BLINK_SLOW_MS, 0);
    } else
#endif
#if HAS_USB
    if (s_powered) {
        if (!s_batt_full) {
            blink(LED_BATT, COL_GREEN, BLINK_SLOW_MS, 0); /* charging */
        }
        /* full & powered -> stay off (complete was a one-shot) */
    } else
#endif
        if (s_batt_low) {
        blink(LED_BATT, COL_RED, BLINK_SLOW_MS, 0);
    }
#if !IS_CENTRAL
    if (!s_unit_connected) {
        blink(LED_UNIT, COL_BLUE, BLINK_SLOW_MS, 0);
    }
#endif
    k_work_reschedule(&tick_work, K_MSEC(CONFIG_CORNIX_STATUS_LED_INTERVAL_MS));
}

/* ---- Event handling ---- */
static int led_event_cb(const zmk_event_t *eh) {
    const struct zmk_battery_state_changed *batt = as_zmk_battery_state_changed(eh);
    if (batt) {
        s_batt_low = batt->state_of_charge <= CONFIG_CORNIX_STATUS_LED_BATTERY_LOW;
#if HAS_USB
        bool full = batt->state_of_charge >= CONFIG_CORNIX_STATUS_LED_BATTERY_FULL;
        /* one green blink when charge completes while on USB power */
        if (s_powered && full && !s_batt_full) {
            blink(LED_BATT, COL_GREEN, BLINK_EVENT_MS, 0);
        }
        s_batt_full = full;
#endif
        return ZMK_EV_EVENT_BUBBLE;
    }

#if HAS_USB
    const struct zmk_usb_conn_state_changed *usb = as_zmk_usb_conn_state_changed(eh);
    if (usb) {
        bool powered = zmk_usb_is_powered();
        /* already-full when plugged in -> show complete once */
        if (powered && !s_powered && s_batt_full) {
            blink(LED_BATT, COL_GREEN, BLINK_EVENT_MS, 0);
        }
        s_powered = powered;
        k_work_reschedule(&tick_work, K_NO_WAIT);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

#if IS_CENTRAL
    if (as_zmk_ble_active_profile_changed(eh)) {
        s_bt_index = zmk_ble_active_profile_index();
        s_bt_connected = zmk_ble_active_profile_is_connected();
        s_bt_open = zmk_ble_active_profile_is_open();
        if (s_bt_connected) {
            blink(LED_BT, chan_color(s_bt_index), BLINK_EVENT_MS, 0);
        }
        k_work_reschedule(&tick_work, K_NO_WAIT);
        return ZMK_EV_EVENT_BUBBLE;
    }
    const struct zmk_split_peripheral_status_changed *ps =
        as_zmk_split_peripheral_status_changed(eh);
    if (ps) {
        s_periph_connected = ps->connected;
        if (s_periph_connected) {
            blink(LED_AUX, COL_BLUE, BLINK_EVENT_MS, 0);
        }
        k_work_reschedule(&tick_work, K_NO_WAIT);
        return ZMK_EV_EVENT_BUBBLE;
    }
#else
    if (as_zmk_split_peripheral_status_changed(eh)) {
        s_unit_connected = zmk_split_bt_peripheral_is_connected();
        if (s_unit_connected) {
            blink(LED_UNIT, COL_BLUE, BLINK_EVENT_MS, 0);
        }
        k_work_reschedule(&tick_work, K_NO_WAIT);
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
        pixels[i] = COL_OFF;
    }
    strip_show();
#if HAS_USB
    s_powered = zmk_usb_is_powered();
#endif
    k_work_init_delayable(&tick_work, tick);
    k_work_reschedule(&tick_work, K_MSEC(1000));
    return 0;
}

SYS_INIT(cornix_status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
