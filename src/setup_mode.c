/*
 * Setup mode + LED indicators for twelvepad.
 *
 * LEDs: LED_0..LED_3 (one per BLE profile) + LED_LOWBAT (led-gpios index 4).
 *
 * Always (every mode / layer):
 *   - battery <= 5 %: LED_LOWBAT blinks (250 ms on / 250 ms off), own timer.
 *     Turned off before deep sleep / soft off.
 *
 * Normal state:
 *   - boot / wake: all profile LEDs on for 500 ms, then only the LED of the
 *     active BLE profile stays on for another 500 ms, then all off
 *   - BLE not connected: the LED of the active profile blinks (500 ms on / 500 ms off)
 *
 * Setup mode (keymap layer 1, entered by the &sm SM_ENTER hold behavior on release):
 *   - all profile LEDs flash with a 250 ms timer
 *   - SM_BATTERY (held): battery bar (1..4 LEDs); on release LEDs off + leave setup mode
 *   - SM_SELECTED_n (after BT_SEL n): leave setup mode, LED of profile n on for 500 ms
 *   - SM_PROFILE (held): LED of the active profile; on release LEDs off + leave setup mode
 *   - SM_SLEEP (on release): animation, then sleep once no key is held
 *       CONFIG_SETUP_MODE_SOFT_OFF=n : ZMK deep sleep (needs CONFIG_ZMK_SLEEP=y)
 *       CONFIG_SETUP_MODE_SOFT_OFF=y : soft off (needs CONFIG_ZMK_PM_SOFT_OFF=y)
 *   - SM_CANCEL (on release) / SM_EXIT (now): leave setup mode
 *   - Sleep failure indicator (all profile LEDs flash, 100 ms):
 *       4 flashes = sleep method not available (e.g. zmk_pm_soft_off not found)
 *       2 flashes = sleep was requested but the system kept running
 *
 * Idle timeout:
 *   CONFIG_SETUP_MODE_SOFT_OFF=n : ZMK's own deep sleep (CONFIG_ZMK_IDLE_SLEEP_TIMEOUT)
 *   CONFIG_SETUP_MODE_SOFT_OFF=y : own idle timer -> soft off
 *
 * Keymap layers: 0 = default, 1 = setup, 2 = sleep (all keys disabled).
 *
 * Power management: when ZMK suspends devices before deep sleep / soft off,
 * the PM hook below stops the timers and turns ALL LEDs off.
 */

#define DT_DRV_COMPAT zmk_behavior_setup_mode

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/ble.h>
#include <zmk/battery.h>
#include <zmk/keymap.h>
#include <zmk/event_manager.h>

#ifdef CONFIG_SETUP_MODE_SOFT_OFF
#include <zmk/events/position_state_changed.h>
#else
#if __has_include(<zmk/events/activity_state_changed.h>)
#include <zmk/events/activity_state_changed.h>
#define HAVE_ACTIVITY_EVENT 1
#endif
#if __has_include(<zmk/activity.h>)
#include <zmk/activity.h>
#endif
#endif

LOG_MODULE_REGISTER(setup_mode, CONFIG_ZMK_LOG_LEVEL);

/* Provided by ZMK when CONFIG_ZMK_PM_SOFT_OFF=y. Weak so a missing symbol
 * does not break the build (soft off then just does not happen). */
extern int zmk_pm_soft_off(void) __attribute__((weak));

/* Can the sleep key do anything with the current configuration? */
#ifdef CONFIG_SETUP_MODE_SOFT_OFF
#define SLEEP_SUPPORTED 1
#elif defined(CONFIG_ZMK_SLEEP) && defined(HAVE_ACTIVITY_EVENT)
#define SLEEP_SUPPORTED 1
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define LED_COUNT 4  /* profile LEDs: LED_0 .. LED_3 */
#define LED_LOWBAT 4 /* index of LED_LOWBAT in led-gpios */
#define LED_TOTAL 5
#define LED_ALL ((1U << LED_COUNT) - 1U)
#define TICK_MS 250
#define SETUP_TICK_MS 250

/* Low battery warning: LED_LOWBAT blinks at 250 ms when soc <= this. */
#define LOW_BATT_PERCENT 5
#define LOWBAT_TICK_MS 250
/* Ignore the battery value right after boot (it may not have been sampled yet). */
#define LOW_BATT_GRACE_MS 5000

#define LAYER_DEFAULT 0
#define LAYER_SETUP 1
#define LAYER_SLEEP 2

/* &sm parameters (keep in sync with twelvepad.keymap) */
enum {
    SM_ENTER = 1,
    SM_EXIT = 2,
    SM_CANCEL = 3,
    SM_BATTERY = 4,
    SM_PROFILE = 5,
    SM_SLEEP = 6,
    SM_SELECTED_0 = 10, /* SM_SELECTED_0 .. SM_SELECTED_0 + 3 */
};

/* BLE profile index -> LED index. Change here to remap. */
static const uint8_t profile_to_led[LED_COUNT] = {0, 1, 2, 3};

static const struct gpio_dt_spec leds[LED_TOTAL] = {
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 0),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 1),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 2),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 3),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 4),
};

/* The key GPIOs owned by the kscan: only read, to check that no key is held. */
#define KEY_GPIO_ELEM(node_id, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx),
static const struct gpio_dt_spec keys[] = {
    DT_FOREACH_PROP_ELEM(DT_NODELABEL(kscan), input_gpios, KEY_GPIO_ELEM)};

static const struct {
    uint8_t mask;
    uint16_t ms;
} sleep_anim[] = {
    {0x0F, 1000}, /* all on     */
    {0x07, 500},  /* LED_3 off  */
    {0x03, 500},  /* LED_2 off  */
    {0x01, 500},  /* LED_1 off  */
    {0x00, 500},  /* LED_0 off  */
};

enum mode {
    MODE_BOOT,
    MODE_NORMAL,
    MODE_SETUP,
    MODE_SETUP_BATT,
    MODE_SETUP_PROFILE,
    MODE_CONFIRM,
    MODE_SLEEP,
    MODE_ERROR,
};

static struct k_work_delayable tick_work;
static struct k_work_delayable lowbat_work;
static enum mode mode = MODE_BOOT;
static uint32_t tick_count;
static bool setup_flash_on;
static bool lowbat_on;
static uint8_t boot_phase;
static bool enter_armed;
static size_t sleep_pos;
static bool sleep_waiting;
static uint8_t err_step;
static uint8_t err_steps;

#ifdef CONFIG_SETUP_MODE_SOFT_OFF
static struct k_work_delayable idle_work;
static int64_t last_activity;
#endif

/* ---- LEDs ---- */

/* Profile LEDs LED_0..LED_3 (bit i = LED_i). LED_LOWBAT is handled separately. */
static void leds_set(uint8_t mask) {
    for (int i = 0; i < LED_COUNT; i++) {
        gpio_pin_set_dt(&leds[i], (mask >> i) & 1U);
    }
}

static void lowbat_set(bool on) {
    lowbat_on = on;
    gpio_pin_set_dt(&leds[LED_LOWBAT], on ? 1 : 0);
}

static void lowbat_start(void) {
    k_work_reschedule(&lowbat_work, K_NO_WAIT);
}

static void lowbat_stop(void) {
    k_work_cancel_delayable(&lowbat_work);
    lowbat_set(false);
}

static void lowbat_handler(struct k_work *work) {
    if (k_uptime_get() > LOW_BATT_GRACE_MS &&
        zmk_battery_state_of_charge() <= LOW_BATT_PERCENT) {
        lowbat_set(!lowbat_on);
    } else {
        lowbat_set(false);
    }
    k_work_reschedule(&lowbat_work, K_MSEC(LOWBAT_TICK_MS));
}

static uint8_t profile_mask(void) {
    uint8_t idx = zmk_ble_active_profile_index();
    if (idx >= LED_COUNT) {
        return 0;
    }
    return BIT(profile_to_led[idx]);
}

/* soc 0..25 -> 1 LED, 26..50 -> 2, 51..75 -> 3, 76..100 -> 4 */
static uint8_t battery_mask(uint8_t soc) {
    uint8_t n;

    if (soc <= 25) {
        n = 1;
    } else if (soc <= 50) {
        n = 2;
    } else if (soc <= 75) {
        n = 3;
    } else {
        n = 4;
    }
    return (uint8_t)((1U << n) - 1U);
}

static bool any_key_pressed(void) {
    for (size_t i = 0; i < ARRAY_SIZE(keys); i++) {
        if (gpio_pin_get_dt(&keys[i]) > 0) {
            return true;
        }
    }
    return false;
}

/* ---- sleep ---- */

/* Start the actual power down. Returns true if it was started (the system is
 * about to go down), false if it is not possible with this configuration. */
static bool power_down(void) {
    leds_set(0);
    lowbat_stop();

#ifdef CONFIG_SETUP_MODE_SOFT_OFF
    if (zmk_pm_soft_off == NULL) {
        LOG_ERR("zmk_pm_soft_off not available");
        return false;
    }
    zmk_pm_soft_off();
    return true;
#elif defined(CONFIG_ZMK_SLEEP) && defined(HAVE_ACTIVITY_EVENT)
    /* Ask ZMK to go to deep sleep right now (same event as its idle timeout). */
    raise_zmk_activity_state_changed(
        (struct zmk_activity_state_changed){.state = ZMK_ACTIVITY_SLEEP});
    return true;
#else
    LOG_ERR("no sleep method available (enable CONFIG_ZMK_SLEEP)");
    return false;
#endif
}

/* ---- modes ---- */

static void go_normal(void) {
    mode = MODE_NORMAL;
    tick_count = 0;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

static void enter_setup(void) {
    zmk_keymap_layer_to(LAYER_SETUP);
    mode = MODE_SETUP;
    setup_flash_on = false;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

static void exit_setup(void) {
    leds_set(0);
    zmk_keymap_layer_to(LAYER_DEFAULT);
    sleep_waiting = false;
    lowbat_start();
    go_normal();
}

/* Leave setup mode and light the LED of profile n for 500 ms. */
static void show_selected(uint8_t profile) {
    zmk_keymap_layer_to(LAYER_DEFAULT);
    mode = MODE_CONFIRM;
    leds_set(BIT(profile_to_led[profile]));
    k_work_reschedule(&tick_work, K_MSEC(500));
}

/* Visible error: all profile LEDs flash `flashes` times (100 ms on / 100 ms off). */
static void show_error(uint8_t flashes) {
    zmk_keymap_layer_to(LAYER_SLEEP); /* keys do nothing while flashing */
    mode = MODE_ERROR;
    err_step = 0;
    err_steps = flashes * 2;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

#ifdef SLEEP_SUPPORTED
static void start_sleep(void) {
    zmk_keymap_layer_to(LAYER_SLEEP);
    mode = MODE_SLEEP;
    sleep_pos = 0;
    sleep_waiting = false;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}
#endif

static void tick_handler(struct k_work *work) {
    switch (mode) {
    case MODE_BOOT:
        if (boot_phase == 0) {
            /* phase 1: keep only the LED of the active BLE profile on */
            boot_phase = 1;
            leds_set(profile_mask());
            k_work_reschedule(&tick_work, K_MSEC(500));
            return;
        }
        leds_set(0);
        go_normal();
        return;

    case MODE_SETUP:
        setup_flash_on = !setup_flash_on;
        leds_set(setup_flash_on ? LED_ALL : 0);
        k_work_reschedule(&tick_work, K_MSEC(SETUP_TICK_MS));
        return;

    case MODE_SETUP_BATT:
        leds_set(battery_mask(zmk_battery_state_of_charge()));
        k_work_reschedule(&tick_work, K_MSEC(500));
        return;

    case MODE_SETUP_PROFILE:
        leds_set(profile_mask());
        k_work_reschedule(&tick_work, K_MSEC(200));
        return;

    case MODE_CONFIRM:
        /* BT_SEL confirmation (500 ms) is over */
        leds_set(0);
        go_normal();
        return;

    case MODE_SLEEP:
#ifdef SLEEP_SUPPORTED
        if (sleep_waiting) {
            /* Power down was requested but the system kept running. */
            LOG_ERR("power down did not happen");
            sleep_waiting = false;
            lowbat_start();
            show_error(2);
            return;
        }
        if (sleep_pos < ARRAY_SIZE(sleep_anim)) {
            leds_set(sleep_anim[sleep_pos].mask);
            k_work_reschedule(&tick_work, K_MSEC(sleep_anim[sleep_pos].ms));
            sleep_pos++;
            return;
        }
        /* Animation finished: only sleep when no key is held, otherwise replay. */
        if (any_key_pressed()) {
            sleep_pos = 0;
            k_work_reschedule(&tick_work, K_NO_WAIT);
            return;
        }
        sleep_waiting = true;
        k_work_reschedule(&tick_work, K_MSEC(2000)); /* fallback if we are still running */
        if (!power_down()) {
            sleep_waiting = false;
            lowbat_start();
            show_error(4);
        }
#else
        exit_setup();
#endif
        return;

    case MODE_ERROR:
        if (err_step >= err_steps) {
            leds_set(0);
            exit_setup();
            return;
        }
        leds_set((err_step % 2) == 0 ? LED_ALL : 0);
        err_step++;
        k_work_reschedule(&tick_work, K_MSEC(100));
        return;

    case MODE_NORMAL:
    default: {
        uint8_t mask = 0;

        /* BLE not connected: LED of the active profile, 500 ms on / 500 ms off */
        if (!zmk_ble_active_profile_is_connected() && ((tick_count / 2) % 2) == 0) {
            mask = profile_mask();
        }
        tick_count++;
        leds_set(mask);
        k_work_reschedule(&tick_work, K_MSEC(TICK_MS));
        return;
    }
    }
}

/* ---- idle timeout -> soft off (only with CONFIG_SETUP_MODE_SOFT_OFF) ---- */

#ifdef CONFIG_SETUP_MODE_SOFT_OFF
static int activity_listener(const zmk_event_t *eh) {
    if (as_zmk_position_state_changed(eh) != NULL) {
        last_activity = k_uptime_get();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(setup_mode_activity, activity_listener);
ZMK_SUBSCRIPTION(setup_mode_activity, zmk_position_state_changed);

static void idle_start(void) {
    last_activity = k_uptime_get();
    k_work_reschedule(&idle_work, K_MSEC(CONFIG_SETUP_MODE_SOFT_OFF_TIMEOUT_MS));
}

static void idle_handler(struct k_work *work) {
    int64_t idle = k_uptime_get() - last_activity;
    int64_t timeout = CONFIG_SETUP_MODE_SOFT_OFF_TIMEOUT_MS;

    if (idle < timeout) {
        k_work_reschedule(&idle_work, K_MSEC(timeout - idle));
        return;
    }
    if (any_key_pressed() || mode != MODE_NORMAL) {
        idle_start();
        return;
    }
    if (!power_down()) {
        lowbat_start();
        show_error(4);
        idle_start(); /* try again later */
    }
}
#endif /* CONFIG_SETUP_MODE_SOFT_OFF */

/* ---- behavior ---- */

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case SM_ENTER:
        /* Hold-tap fired its hold action (> 500 ms). Enter on release. */
        enter_armed = (mode == MODE_NORMAL);
        break;
    case SM_EXIT:
        if (mode == MODE_SETUP) {
            exit_setup();
        }
        break;
    case SM_BATTERY:
        if (mode == MODE_SETUP) {
            mode = MODE_SETUP_BATT;
            k_work_reschedule(&tick_work, K_NO_WAIT);
        }
        break;
    case SM_PROFILE:
        if (mode == MODE_SETUP) {
            mode = MODE_SETUP_PROFILE;
            k_work_reschedule(&tick_work, K_NO_WAIT);
        }
        break;
    default:
        if (binding->param1 >= SM_SELECTED_0 && binding->param1 < SM_SELECTED_0 + LED_COUNT) {
            if (mode == MODE_SETUP) {
                show_selected(binding->param1 - SM_SELECTED_0);
            }
        }
        break;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case SM_ENTER:
        if (enter_armed && mode == MODE_NORMAL) {
            enter_setup();
        }
        enter_armed = false;
        break;
    case SM_CANCEL:
        if (mode == MODE_SETUP) {
            exit_setup();
        }
        break;
    case SM_BATTERY:
        if (mode == MODE_SETUP_BATT) {
            exit_setup();
        }
        break;
    case SM_PROFILE:
        if (mode == MODE_SETUP_PROFILE) {
            exit_setup();
        }
        break;
    case SM_SLEEP:
        if (mode == MODE_SETUP) {
#ifdef SLEEP_SUPPORTED
            start_sleep();
#else
            exit_setup(); /* no sleep method configured: just leave setup mode */
#endif
        }
        break;
    default:
        break;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

/* Called by ZMK when it suspends devices (idle deep sleep, soft off). */
static int __maybe_unused setup_mode_pm_action(const struct device *dev,
                                               enum pm_device_action action) {
    switch (action) {
    case PM_DEVICE_ACTION_SUSPEND:
        k_work_cancel_delayable(&tick_work);
#ifdef CONFIG_SETUP_MODE_SOFT_OFF
        k_work_cancel_delayable(&idle_work);
#endif
        leds_set(0);
        lowbat_stop();
        return 0;
    case PM_DEVICE_ACTION_RESUME:
        /* Normally the keyboard reboots on wake; if we resume without a reboot,
         * start again from a clean default state. */
        exit_setup();
#ifdef CONFIG_SETUP_MODE_SOFT_OFF
        idle_start();
#endif
        return 0;
    default:
        return -ENOTSUP;
    }
}

PM_DEVICE_DT_INST_DEFINE(0, setup_mode_pm_action);

static const struct behavior_driver_api setup_mode_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, PM_DEVICE_DT_INST_GET(0), NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &setup_mode_driver_api);

static int setup_mode_init(void) {
    for (int i = 0; i < LED_TOTAL; i++) {
        if (!gpio_is_ready_dt(&leds[i])) {
            LOG_ERR("LED %d gpio not ready", i);
            return -ENODEV;
        }
        int ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_ERR("LED %d configure failed: %d", i, ret);
            return ret;
        }
    }

    k_work_init_delayable(&tick_work, tick_handler);
    k_work_init_delayable(&lowbat_work, lowbat_handler);
#ifdef CONFIG_SETUP_MODE_SOFT_OFF
    k_work_init_delayable(&idle_work, idle_handler);
    idle_start();
#endif

    /* Low battery LED runs in every mode. */
    lowbat_start();

    /* Boot / wake: all profile LEDs on for 500 ms (phase 0), then the active
     * profile's LED for 500 ms (phase 1), then off (MODE_BOOT handler). */
    mode = MODE_BOOT;
    boot_phase = 0;
    leds_set(LED_ALL);
    k_work_reschedule(&tick_work, K_MSEC(500));
    return 0;
}

SYS_INIT(setup_mode_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
