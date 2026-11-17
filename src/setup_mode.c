/*
 * Setup mode + low battery LED (+ optional profile LEDs) for swanboard.
 *
 * Always:
 *   - LED_LOWBAT (lowbat-gpios, P0.15): battery <= 5 %: blinks 250 ms on / 250 ms off.
 *     Works in every mode. Turned off before deep sleep / soft off.
 *   - Setup mode (keymap layer 1 "SETTING", entered by the &sm SM_ENTER hold behavior
 *     on release): SM_EXIT (now) / SM_CANCEL (on release) leave it.
 *   - SM_SLEEP (on release): layer 2 "POWER", then sleep once no key is held
 *       CONFIG_SETUP_MODE_SOFT_OFF=n : ZMK deep sleep (needs CONFIG_ZMK_SLEEP=y)
 *       CONFIG_SETUP_MODE_SOFT_OFF=y : soft off (needs CONFIG_ZMK_PM_SOFT_OFF=y);
 *                                      nothing wakes it except the reset button
 *   - Idle timeout:
 *       CONFIG_SETUP_MODE_SOFT_OFF=n : ZMK's own deep sleep (CONFIG_ZMK_IDLE_SLEEP_TIMEOUT)
 *       CONFIG_SETUP_MODE_SOFT_OFF=y : own idle timer -> soft off
 *                                      (CONFIG_SETUP_MODE_SOFT_OFF_TIMEOUT_MS=0 disables it,
 *                                       so ZMK's deep sleep can handle the idle timeout)
 *
 * With CONFIG_SETUP_MODE_PROFILE_LEDS=y (swanboard_led shield, profile-led-gpios):
 *   - boot / wake: all profile LEDs on for 500 ms, then only the LED of the active
 *     BLE profile for another 500 ms, then all off
 *   - BLE not connected: the LED of the active profile blinks (500 ms / 500 ms)
 *   - setup mode: all profile LEDs flash (250 ms)
 *   - SM_BATTERY (held): battery bar (1..4 LEDs), release = LEDs off + leave setup mode
 *   - SM_PROFILE (held): LED of the active profile, release = LEDs off + leave setup mode
 *   - SM_SELECTED_n (after BT_SEL n): leave setup mode, LED of profile n on for 500 ms
 *   - sleep: LED animation (all on 1 s, then LED_3 -> LED_0 off, 500 ms each)
 * Without it (nice!view): the display shows all of that, SM_BATTERY / SM_PROFILE / SM_SELECTED_n
 * simply leave setup mode, and the sleep waits 1 s while the "POWER" layer is shown.
 *
 * Keymap layers: 0 = default, 1 = setup, 2 = sleep (all keys disabled).
 *
 * Power management: when ZMK suspends devices before deep sleep / soft off,
 * the PM hook below stops the timers and turns all LEDs off.
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

/* Provided by ZMK when CONFIG_ZMK_PM_SOFT_OFF=y. Declared weak so a build without
 * soft off still links (power_down() then reports failure and we stay awake). */
extern int zmk_pm_soft_off(void) __attribute__((weak));

/* Can the sleep key do anything with the current configuration? */
#ifdef CONFIG_SETUP_MODE_SOFT_OFF
#define SLEEP_SUPPORTED 1
#elif defined(CONFIG_ZMK_SLEEP) && defined(HAVE_ACTIVITY_EVENT)
#define SLEEP_SUPPORTED 1
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

/* Low battery warning */
#define LOW_BATT_PERCENT 5
#define LOWBAT_BLINK_MS 250    /* blink interval while the battery is low */
#define LOWBAT_CHECK_MS 5000   /* how often to look at the battery otherwise */
#define LOW_BATT_GRACE_MS 5000 /* ignore the battery value right after boot */

#define TICK_MS 250
#define SETUP_TICK_MS 250

#define LAYER_DEFAULT 0
#define LAYER_SETUP 1
#define LAYER_SLEEP 2

/* &sm parameters (keep in sync with swanboard.keymap) */
enum {
    SM_ENTER = 1,
    SM_EXIT = 2,
    SM_CANCEL = 3,
    SM_BATTERY = 4,
    SM_PROFILE = 5,
    SM_SLEEP = 6,
    SM_SELECTED_0 = 10, /* SM_SELECTED_0 .. SM_SELECTED_0 + 3 */
};

static const struct gpio_dt_spec lowbat = GPIO_DT_SPEC_INST_GET_BY_IDX(0, lowbat_gpios, 0);

#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
#define LED_COUNT 4 /* profile LEDs: LED_0 .. LED_3 */
#define LED_ALL ((1U << LED_COUNT) - 1U)

/* BLE profile index -> LED index. Change here to remap. */
static const uint8_t profile_to_led[LED_COUNT] = {0, 1, 2, 3};

static const struct gpio_dt_spec profile_leds[LED_COUNT] = {
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, profile_led_gpios, 0),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, profile_led_gpios, 1),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, profile_led_gpios, 2),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, profile_led_gpios, 3),
};
#else
#define LED_COUNT 4 /* only used to range-check SM_SELECTED_n */
#endif

/* The key GPIOs owned by the kscan: only read, to check that no key is held. */
#define KEY_GPIO_ELEM(node_id, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx),
static const struct gpio_dt_spec keys[] = {
    DT_FOREACH_PROP_ELEM(DT_NODELABEL(kscan), input_gpios, KEY_GPIO_ELEM)};

struct anim_step {
    uint8_t mask;
    uint16_t ms;
};

#ifdef SLEEP_SUPPORTED
static const struct anim_step sleep_anim[] = {
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
    {0x0F, 1000}, /* all on     */
    {0x07, 500},  /* LED_3 off  */
    {0x03, 500},  /* LED_2 off  */
    {0x01, 500},  /* LED_1 off  */
    {0x00, 500},  /* LED_0 off  */
#else
    {0x00, 1000}, /* "POWER" layer is shown on the display */
#endif
};
#endif

enum mode {
    MODE_BOOT,
    MODE_NORMAL,
    MODE_SETUP,
    MODE_SETUP_BATT,
    MODE_SETUP_PROFILE,
    MODE_CONFIRM,
    MODE_SLEEP,
};

static struct k_work_delayable tick_work;
static struct k_work_delayable lowbat_work;
static enum mode mode = MODE_BOOT;
static bool lowbat_on;
static bool enter_armed;
#ifdef SLEEP_SUPPORTED
static size_t sleep_pos;
static bool sleep_waiting;
#endif
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
static uint32_t tick_count;
static bool setup_flash_on;
static uint8_t boot_phase;
#endif

#ifdef CONFIG_SETUP_MODE_SOFT_OFF
static struct k_work_delayable idle_work;
static int64_t last_activity;
#endif

/* ---- LEDs ---- */

/* Profile LEDs LED_0..LED_3 (bit i = LED_i). Does nothing without profile LEDs. */
static void leds_set(uint8_t mask) {
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
    for (int i = 0; i < LED_COUNT; i++) {
        gpio_pin_set_dt(&profile_leds[i], (mask >> i) & 1U);
    }
#else
    (void)mask;
#endif
}

static void lowbat_set(bool on) {
    lowbat_on = on;
    gpio_pin_set_dt(&lowbat, on ? 1 : 0);
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
        k_work_reschedule(&lowbat_work, K_MSEC(LOWBAT_BLINK_MS));
    } else {
        lowbat_set(false);
        k_work_reschedule(&lowbat_work, K_MSEC(LOWBAT_CHECK_MS));
    }
}

#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
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
#endif

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
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
    tick_count = 0;
    k_work_reschedule(&tick_work, K_NO_WAIT);
#endif
}

static void enter_setup(void) {
    zmk_keymap_layer_to(LAYER_SETUP);
    mode = MODE_SETUP;
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
    setup_flash_on = false;
    k_work_reschedule(&tick_work, K_NO_WAIT);
#endif
}

static void exit_setup(void) {
    leds_set(0);
    zmk_keymap_layer_to(LAYER_DEFAULT);
#ifdef SLEEP_SUPPORTED
    sleep_waiting = false;
#endif
    lowbat_start();
    go_normal();
}

#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
/* Leave setup mode and light the LED of profile n for 500 ms. */
static void show_selected(uint8_t profile) {
    zmk_keymap_layer_to(LAYER_DEFAULT);
    mode = MODE_CONFIRM;
    leds_set(BIT(profile_to_led[profile]));
    k_work_reschedule(&tick_work, K_MSEC(500));
}
#endif

#ifdef SLEEP_SUPPORTED
static void start_sleep(void) {
    zmk_keymap_layer_to(LAYER_SLEEP); /* keys do nothing, display shows "POWER" */
    mode = MODE_SLEEP;
    sleep_pos = 0;
    sleep_waiting = false;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}
#endif

static void tick_handler(struct k_work *work) {
    switch (mode) {
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
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
#endif

    case MODE_SLEEP:
#ifdef SLEEP_SUPPORTED
        if (sleep_waiting) {
            /* Power down was requested but the system kept running. */
            LOG_ERR("power down did not happen");
            exit_setup();
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
            exit_setup(); /* sleep not possible: back to normal operation */
        }
#else
        exit_setup();
#endif
        return;

    case MODE_NORMAL:
    default:
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
    {
        uint8_t mask = 0;

        /* BLE not connected: LED of the active profile, 500 ms on / 500 ms off */
        if (!zmk_ble_active_profile_is_connected() && ((tick_count / 2) % 2) == 0) {
            mask = profile_mask();
        }
        tick_count++;
        leds_set(mask);
        k_work_reschedule(&tick_work, K_MSEC(TICK_MS));
    }
#endif
        return;
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
    /* timeout 0 = no idle soft off (ZMK's own deep sleep timer can be used instead) */
    if (CONFIG_SETUP_MODE_SOFT_OFF_TIMEOUT_MS > 0) {
        k_work_reschedule(&idle_work, K_MSEC(CONFIG_SETUP_MODE_SOFT_OFF_TIMEOUT_MS));
    }
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
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
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
#endif
    default:
        if (binding->param1 >= SM_SELECTED_0 && binding->param1 < SM_SELECTED_0 + LED_COUNT) {
            if (mode == MODE_SETUP) {
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
                show_selected(binding->param1 - SM_SELECTED_0);
#else
                exit_setup(); /* the display shows the new profile */
#endif
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
    case SM_PROFILE:
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
        if (mode == MODE_SETUP_BATT || mode == MODE_SETUP_PROFILE) {
            exit_setup();
        }
#else
        if (mode == MODE_SETUP) {
            exit_setup(); /* nothing to show: just leave setup mode */
        }
#endif
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

static int config_output(const struct gpio_dt_spec *spec) {
    if (!gpio_is_ready_dt(spec)) {
        LOG_ERR("LED gpio not ready");
        return -ENODEV;
    }
    int ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("LED configure failed: %d", ret);
    }
    return ret;
}

static int setup_mode_init(void) {
    int ret = config_output(&lowbat);
    if (ret < 0) {
        return ret;
    }
#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
    for (int i = 0; i < LED_COUNT; i++) {
        ret = config_output(&profile_leds[i]);
        if (ret < 0) {
            return ret;
        }
    }
#endif

    k_work_init_delayable(&tick_work, tick_handler);
    k_work_init_delayable(&lowbat_work, lowbat_handler);
#ifdef CONFIG_SETUP_MODE_SOFT_OFF
    k_work_init_delayable(&idle_work, idle_handler);
    idle_start();
#endif

    lowbat_start();

#ifdef CONFIG_SETUP_MODE_PROFILE_LEDS
    /* Boot / wake: all profile LEDs on for 500 ms (phase 0), then the active
     * profile's LED for 500 ms (phase 1), then off (MODE_BOOT handler). */
    mode = MODE_BOOT;
    boot_phase = 0;
    leds_set(LED_ALL);
    k_work_reschedule(&tick_work, K_MSEC(500));
#else
    mode = MODE_NORMAL;
#endif
    return 0;
}

SYS_INIT(setup_mode_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
