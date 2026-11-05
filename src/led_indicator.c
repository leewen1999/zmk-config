/*
 * 4-LED indicator for twelvepad.
 *
 *  - Boot / wake:      all LEDs on for 500 ms, then off.
 *  - Not connected:    the LED of the active BLE profile blinks (500 ms).
 *  - &led_ind 1 held:  the LED of the active BLE profile stays on.
 *  - &led_ind 2 held:  battery level bar (1..4 LEDs).
 *  - &led_ind 3:       sleep animation (all on 1 s, then LED_3 -> LED_0 off, 500 ms each).
 */

#define DT_DRV_COMPAT zmk_behavior_led_indicator

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/ble.h>
#include <zmk/battery.h>

LOG_MODULE_REGISTER(led_indicator, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define LED_COUNT 4
#define LED_ALL ((1U << LED_COUNT) - 1U)
#define TICK_MS 250

enum {
    PARAM_SHOW_PROFILE = 1,
    PARAM_SHOW_BATTERY = 2,
    PARAM_SLEEP_ANIM = 3,
};

/* BLE profile index -> LED index. Change here to remap. */
static const uint8_t profile_to_led[LED_COUNT] = {0, 1, 2, 3};

static const struct gpio_dt_spec leds[LED_COUNT] = {
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 0),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 1),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 2),
    GPIO_DT_SPEC_INST_GET_BY_IDX(0, led_gpios, 3),
};

struct anim_step {
    uint8_t mask;
    uint16_t ms;
};

static const struct anim_step boot_anim[] = {
    {LED_ALL, 500},
};

static const struct anim_step sleep_anim[] = {
    {0x0F, 1000}, /* all on          */
    {0x07, 500},  /* LED_3 off       */
    {0x03, 500},  /* LED_2 off       */
    {0x01, 500},  /* LED_1 off       */
    {0x00, 500},  /* LED_0 off       */
    {0x00, 2000}, /* stay off until soft off kicks in */
};

enum mode {
    MODE_NORMAL,
    MODE_SHOW_PROFILE,
    MODE_SHOW_BATTERY,
    MODE_ANIM,
};

static struct k_work_delayable tick_work;
static enum mode mode = MODE_NORMAL;
static uint32_t tick_count;
static const struct anim_step *anim;
static size_t anim_len;
static size_t anim_pos;

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

static uint8_t battery_mask(void) {
    uint8_t soc = zmk_battery_state_of_charge();
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

static void go_normal(void) {
    mode = MODE_NORMAL;
    tick_count = 0;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

static void start_anim(const struct anim_step *steps, size_t len) {
    mode = MODE_ANIM;
    anim = steps;
    anim_len = len;
    anim_pos = 0;
    k_work_reschedule(&tick_work, K_NO_WAIT);
}

static void tick_handler(struct k_work *work) {
    switch (mode) {
    case MODE_ANIM:
        if (anim_pos >= anim_len) {
            leds_set(0);
            go_normal();
            return;
        }
        leds_set(anim[anim_pos].mask);
        k_work_reschedule(&tick_work, K_MSEC(anim[anim_pos].ms));
        anim_pos++;
        return;

    case MODE_SHOW_PROFILE:
        leds_set(profile_mask());
        k_work_reschedule(&tick_work, K_MSEC(200));
        return;

    case MODE_SHOW_BATTERY:
        leds_set(battery_mask());
        k_work_reschedule(&tick_work, K_MSEC(500));
        return;

    case MODE_NORMAL:
    default:
        if (zmk_ble_active_profile_is_connected()) {
            leds_set(0);
            tick_count = 0;
        } else {
            /* 2 ticks on, 2 ticks off = 500 ms / 500 ms */
            bool on = ((tick_count++ / 2) % 2) == 0;
            leds_set(on ? profile_mask() : 0);
        }
        k_work_reschedule(&tick_work, K_MSEC(TICK_MS));
        return;
    }
}

static int on_pressed(struct zmk_behavior_binding *binding,
                      struct zmk_behavior_binding_event event) {
    if (mode == MODE_ANIM) {
        return ZMK_BEHAVIOR_OPAQUE;
    }

    switch (binding->param1) {
    case PARAM_SHOW_PROFILE:
        mode = MODE_SHOW_PROFILE;
        k_work_reschedule(&tick_work, K_NO_WAIT);
        break;
    case PARAM_SHOW_BATTERY:
        mode = MODE_SHOW_BATTERY;
        k_work_reschedule(&tick_work, K_NO_WAIT);
        break;
    case PARAM_SLEEP_ANIM:
        start_anim(sleep_anim, ARRAY_SIZE(sleep_anim));
        break;
    default:
        LOG_WRN("Unknown param %d", binding->param1);
        break;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_released(struct zmk_behavior_binding *binding,
                       struct zmk_behavior_binding_event event) {
    if ((binding->param1 == PARAM_SHOW_PROFILE && mode == MODE_SHOW_PROFILE) ||
        (binding->param1 == PARAM_SHOW_BATTERY && mode == MODE_SHOW_BATTERY)) {
        leds_set(0);
        go_normal();
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api led_indicator_driver_api = {
    .binding_pressed = on_pressed,
    .binding_released = on_released,
};

BEHAVIOR_DT_INST_DEFINE(0, NULL, NULL, NULL, NULL, POST_KERNEL,
                        CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &led_indicator_driver_api);

static int led_indicator_init(void) {
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
    start_anim(boot_anim, ARRAY_SIZE(boot_anim));
    return 0;
}

SYS_INIT(led_indicator_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
