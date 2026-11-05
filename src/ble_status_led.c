/*
 * Blink an LED (default P0.15, the nice!nano onboard blue LED) while the
 * active BLE profile is NOT connected. LED is off when connected.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zmk/ble.h>

LOG_MODULE_REGISTER(ble_status_led, CONFIG_ZMK_LOG_LEVEL);

static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static struct k_work_delayable blink_work;
static bool led_on;

static void set_led(bool on) {
    led_on = on;
    int level = on ? 1 : 0;
    if (IS_ENABLED(CONFIG_BLE_STATUS_LED_ACTIVE_LOW)) {
        level = !level;
    }
    gpio_pin_set_raw(gpio_dev, CONFIG_BLE_STATUS_LED_PIN, level);
}

static void blink_handler(struct k_work *work) {
    if (zmk_ble_active_profile_is_connected()) {
        set_led(false);
        k_work_schedule(&blink_work, K_MSEC(CONFIG_BLE_STATUS_LED_CHECK_MS));
    } else {
        set_led(!led_on);
        k_work_schedule(&blink_work, K_MSEC(CONFIG_BLE_STATUS_LED_BLINK_MS));
    }
}

static int ble_status_led_init(void) {
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("gpio0 not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure(gpio_dev, CONFIG_BLE_STATUS_LED_PIN, GPIO_OUTPUT);
    if (ret < 0) {
        LOG_ERR("LED pin configure failed: %d", ret);
        return ret;
    }

    set_led(false);
    k_work_init_delayable(&blink_work, blink_handler);
    k_work_schedule(&blink_work, K_MSEC(CONFIG_BLE_STATUS_LED_BLINK_MS));
    return 0;
}

SYS_INIT(ble_status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
