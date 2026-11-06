/*
 * Setup mode + 4-LED indicator for twelvepad.
 *
 * Normal state:
 *   - boot / wake: all LEDs on for 500 ms, then only the LED of the active
 *     BLE profile stays on for another 500 ms, then all off
 *   - BLE not connected: the LED of the active profile blinks (500 ms on / 500 ms off)
 *   - battery <= 5 %: LED_0 blinks (250 ms on / 250 ms off), checked every tick
 *
 * Setup mode (keymap layer 1, entered by the &sm SM_ENTER hold behavior on release):
 *   - all LEDs flash with a 250 ms timer
 *   - SM_BATTERY (held): battery bar (1..4 LEDs, 0-5 %: LED_0 blinks at 500 ms);
 *                        on release LEDs off + leave setup mode
 *   - SM_SELECTED_n (after BT_SEL n): leave setup mode, LED of profile n on for 500 ms
 *   - SM_PROFILE (held): LED of the active profile; on release LEDs off + leave setup mode
 *   - SM_SLEEP (on release): animation, then soft off once no key is held
 *   - SM_CANCEL (on release) / SM_EXIT (now): leave setup mode
 *
 * Keymap layers: 0 = default, 1 = setup, 2 = sleep (all keys disabled).
 *
 * Power management: when ZMK suspends devices before deep sleep / soft off,
 * the PM hook below stops the timer and turns all LEDs off (otherwise a lit
 * LED stays lit during deep sleep).
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

LOG_MODULE_REGISTER(setup_mode, CONFIG_ZMK_LOG_LEVEL);

/* Provided by ZMK when CONFIG_ZMK_PM_SOFT_OFF=y. Weak so a missing symbol
 * does not break the build (the sleep sequence then just returns to normal). */
extern int zmk_pm_soft_off(void) __attribute__((weak));

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define LED_COUNT 4
#define LED_ALL ((1U << LED_COUNT) - 1U)
#define TICK_MS 250
#define SETUP_TICK_MS 250

/* Low battery warning (default layer): LED_0 blinks at 250 ms when soc <= this. */
#define LOW_BATT_PERCENT 5
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

static const struct gpio_dt_spec leds[LED_COUNT] = {
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 0),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 1),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 2),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 3),
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
};

static struct k_work_delayable tick_work;
static enum mode mode = MODE_BOOT;
static uint32_t tick_count;
static bool setup_flash_on;
static bool batt_blink_on;
static uint8_t boot_phase;
static bool enter_armed;
static size_t sleep_pos;
static bool sleep_waiting;

static void leds_set(uint8_t mask) {
    for (int i = 0; i < LED_COUNT; i++) {
        gpio_pin_set_dt(&leds[i], (mask >> i) & 1U);
    }
}

static uint8_t profile_mask(void) {
    uint8_t idx = zmk_ble_active_profile_index();
    if (idx >= LED_COUNT) {
        return 0;
    }
    return BIT(profile_to_led[idx]);
}

/* soc 6..100 -> 1..4 solid LEDs (0..5 % is handled separately: blinking) */
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
    go_normal();
}

/* Leave setup mode and light the LED of profile n for 500 ms. */
static void show_selected(uint8_t profile) {
    zmk_keymap_layer_to(LAYER_DEFAULT);
    mode = MODE_CONFIRM;
    leds_set(BIT(profile_to_led[profile]));
    k_work_reschedule(&tick_work, K_MSEC(500));
}

static void start_sleep(void) {
    zmk_keymap_layer_to(LAYER_SLEEP);
    mode = MODE_SLEEP;
    sleep_pos = 0;
    sleep_waiting = false;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

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

    case MODE_SETUP_BATT: {
        uint8_t soc = zmk_battery_state_of_charge();

        if (soc <= LOW_BATT_PERCENT) {
            /* critical: LED_0 blinks, 250 ms on / 250 ms off */
            batt_blink_on = !batt_blink_on;
            leds_set(batt_blink_on ? 0x01 : 0x00);
            k_work_reschedule(&tick_work, K_MSEC(250));
        } else {
            leds_set(battery_mask(soc));
            k_work_reschedule(&tick_work, K_MSEC(500));
        }
        return;
    }

    case MODE_CONFIRM:
        /* BT_SEL confirmation (500 ms) is over */
        leds_set(0);
        go_normal();
        return;

    case MODE_SETUP_PROFILE:
        leds_set(profile_mask());
        k_work_reschedule(&tick_work, K_MSEC(200));
        return;

    case MODE_SLEEP:
        if (sleep_waiting) {
            /* Soft off did not happen: go back to normal operation. */
            LOG_ERR("soft off did not happen");
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
        leds_set(0);
        if (zmk_pm_soft_off == NULL) {
            LOG_ERR("zmk_pm_soft_off not available");
            exit_setup();
            return;
        }
        sleep_waiting = true;
        k_work_reschedule(&tick_work, K_MSEC(2000)); /* fallback if we are still running */
        zmk_pm_soft_off();
        return;

    case MODE_NORMAL:
    default: {
        uint8_t mask = 0;

        /* BLE not connected: LED of the active profile, 500 ms on / 500 ms off */
        if (!zmk_ble_active_profile_is_connected() && ((tick_count / 2) % 2) == 0) {
            mask |= profile_mask();
        }

        /* Low battery: LED_0 blinks 250 ms on / 250 ms off (overrides LED_0 above) */
        if (k_uptime_get() > LOW_BATT_GRACE_MS &&
            zmk_battery_state_of_charge() <= LOW_BATT_PERCENT) {
            mask &= (uint8_t)~0x01U;
            if ((tick_count % 2) == 0) {
                mask |= 0x01;
            }
        }

        tick_count++;
        leds_set(mask);
        k_work_reschedule(&tick_work, K_MSEC(TICK_MS));
        return;
    }
    }
}

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
            batt_blink_on = false;
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
            start_sleep();
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
        leds_set(0);
        return 0;
    case PM_DEVICE_ACTION_RESUME:
        /* Normally the keyboard reboots on wake; if we resume without a reboot,
         * start again from a clean default state. */
        exit_setup();
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
    for (int i = 0; i < LED_COUNT; i++) {
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

    /* Boot / wake: all LEDs on for 500 ms (phase 0), then the active profile's
     * LED for 500 ms (phase 1), then off (MODE_BOOT handler). */
    mode = MODE_BOOT;
    boot_phase = 0;
    leds_set(LED_ALL);
    k_work_reschedule(&tick_work, K_MSEC(500));
    return 0;
}

SYS_INIT(setup_mode_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
