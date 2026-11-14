/*
 * Setup mode + low battery LED for twelvepad (nice!view version).
 *
 * The nice!view shows the BLE profile, battery and the active layer name, so
 * the profile / battery LEDs of the LED version are gone. What is left:
 *
 *   - LED_LOWBAT (P0.15, lowbat-gpios): battery <= 5 %: blinks 250 ms on / 250 ms off.
 *     Works in every mode. Turned off before deep sleep / soft off.
 *
 *   - Setup mode (keymap layer 1 "Setup", entered by the &sm SM_ENTER hold behavior
 *     on release):
 *       SM_EXIT   (now)        : leave setup mode
 *       SM_CANCEL (on release) : leave setup mode
 *       SM_SLEEP  (on release) : layer 2 "Sleep" for 1 s, then sleep once no key is held
 *           CONFIG_SETUP_MODE_SOFT_OFF=n : ZMK deep sleep (needs CONFIG_ZMK_SLEEP=y)
 *           CONFIG_SETUP_MODE_SOFT_OFF=y : soft off (needs CONFIG_ZMK_PM_SOFT_OFF=y);
 *                                          nothing wakes it except the reset button
 *
 *   - Idle timeout:
 *       CONFIG_SETUP_MODE_SOFT_OFF=n : ZMK's own deep sleep (CONFIG_ZMK_IDLE_SLEEP_TIMEOUT)
 *       CONFIG_SETUP_MODE_SOFT_OFF=y : own idle timer -> soft off
 *                                      (CONFIG_SETUP_MODE_SOFT_OFF_TIMEOUT_MS=0 disables it,
 *                                       so ZMK's deep sleep can handle the idle timeout)
 *
 * Keymap layers: 0 = default, 1 = setup, 2 = sleep (all keys disabled).
 *
 * Power management: when ZMK suspends devices before deep sleep / soft off,
 * the PM hook below stops the timers and turns the LED off.
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
#define LOWBAT_BLINK_MS 250       /* blink interval while the battery is low */
#define LOWBAT_CHECK_MS 5000      /* how often to look at the battery otherwise */
#define LOW_BATT_GRACE_MS 5000    /* ignore the battery value right after boot */

/* Sleep sequence */
#define SLEEP_SHOW_MS 1000        /* "Sleep" layer shown on the display before power down */

#define LAYER_DEFAULT 0
#define LAYER_SETUP 1
#define LAYER_SLEEP 2

/* &sm parameters (keep in sync with twelvepad.keymap) */
enum {
    SM_ENTER = 1,
    SM_EXIT = 2,
    SM_CANCEL = 3,
    SM_SLEEP = 6,
};

static const struct gpio_dt_spec lowbat = GPIO_DT_SPEC_INST_GET_BY_IDX(0, lowbat_gpios, 0);

/* The key GPIOs owned by the kscan: only read, to check that no key is held. */
#define KEY_GPIO_ELEM(node_id, prop, idx) GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx),
static const struct gpio_dt_spec keys[] = {
    DT_FOREACH_PROP_ELEM(DT_NODELABEL(kscan), input_gpios, KEY_GPIO_ELEM)};

enum mode {
    MODE_NORMAL,
    MODE_SETUP,
    MODE_SLEEP,
};

static struct k_work_delayable lowbat_work;
#ifdef SLEEP_SUPPORTED
static struct k_work_delayable sleep_work;
#endif
static enum mode mode = MODE_NORMAL;
static bool lowbat_on;
static bool enter_armed;
static bool sleep_waiting;

#ifdef CONFIG_SETUP_MODE_SOFT_OFF
static struct k_work_delayable idle_work;
static int64_t last_activity;
#endif

/* ---- low battery LED ---- */

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

static void enter_setup(void) {
    zmk_keymap_layer_to(LAYER_SETUP);
    mode = MODE_SETUP;
}

static void exit_setup(void) {
    zmk_keymap_layer_to(LAYER_DEFAULT);
    mode = MODE_NORMAL;
    sleep_waiting = false;
    lowbat_start();
}

#ifdef SLEEP_SUPPORTED
static void start_sleep(void) {
    zmk_keymap_layer_to(LAYER_SLEEP); /* keys do nothing, display shows "Sleep" */
    mode = MODE_SLEEP;
    sleep_waiting = false;
    k_work_reschedule(&sleep_work, K_MSEC(SLEEP_SHOW_MS));
}

static void sleep_handler(struct k_work *work) {
    if (sleep_waiting) {
        /* Power down was requested but the system kept running. */
        LOG_ERR("power down did not happen");
        exit_setup();
        return;
    }
    /* Only sleep when no key is held, otherwise wait another round. */
    if (any_key_pressed()) {
        k_work_reschedule(&sleep_work, K_MSEC(SLEEP_SHOW_MS));
        return;
    }
    sleep_waiting = true;
    k_work_reschedule(&sleep_work, K_MSEC(2000)); /* fallback if we are still running */
    if (!power_down()) {
        exit_setup(); /* sleep not possible: back to normal operation */
    }
}
#endif

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
    default:
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
#ifdef SLEEP_SUPPORTED
        k_work_cancel_delayable(&sleep_work);
#endif
#ifdef CONFIG_SETUP_MODE_SOFT_OFF
        k_work_cancel_delayable(&idle_work);
#endif
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
    if (!gpio_is_ready_dt(&lowbat)) {
        LOG_ERR("low battery LED gpio not ready");
        return -ENODEV;
    }
    int ret = gpio_pin_configure_dt(&lowbat, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("low battery LED configure failed: %d", ret);
        return ret;
    }

    k_work_init_delayable(&lowbat_work, lowbat_handler);
#ifdef SLEEP_SUPPORTED
    k_work_init_delayable(&sleep_work, sleep_handler);
#endif
#ifdef CONFIG_SETUP_MODE_SOFT_OFF
    k_work_init_delayable(&idle_work, idle_handler);
    idle_start();
#endif

    lowbat_start();
    return 0;
}

SYS_INIT(setup_mode_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
