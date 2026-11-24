/*
 * Startup beep: plays the melody in notes[] on a piezo buzzer at boot.
 * Waking up from deep sleep / soft off is a reboot, so it plays then as well.
 *
 * The tone is a square wave made by toggling the GPIO from a kernel timer (any pin
 * works, no PWM peripheral needed). The pin is driven low when a note ends and after
 * the last one.
 */

#define DT_DRV_COMPAT swanboard_startup_beep

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(startup_beep, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct note {
    uint16_t freq_hz; /* 0 = rest (silence) */
    uint16_t ms;      /* duration in milliseconds */
};

/* The melody: { frequency in Hz, duration in ms }. Edit this array to change it. */
static const struct note notes[] = {
    {650, 60},  /* beep  (650 Hz, 0.06 s) */
    {0, 30},    /* rest  (0.03 s)         */
    {650, 500}, /* beep  (650 Hz, 0.5 s)  */
};

static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_INST_GET(0, buzzer_gpios);

static struct k_timer tone_timer;
static struct k_work_delayable note_work;
static size_t note_idx;

static void tone_tick(struct k_timer *timer) {
    gpio_pin_toggle_dt(&buzzer);
}

static void silence(void) {
    k_timer_stop(&tone_timer);
    gpio_pin_set_dt(&buzzer, 0);
}

static void note_handler(struct k_work *work) {
    silence(); /* end of the previous note */

    if (note_idx >= ARRAY_SIZE(notes)) {
        return; /* melody finished, the pin stays low */
    }

    const struct note *n = &notes[note_idx++];

    if (n->freq_hz > 0) {
        uint32_t half_period_us = 500000U / n->freq_hz;

        k_timer_start(&tone_timer, K_USEC(half_period_us), K_USEC(half_period_us));
    }
    k_work_schedule(&note_work, K_MSEC(n->ms));
}

static int startup_beep_init(void) {
    if (!gpio_is_ready_dt(&buzzer)) {
        LOG_ERR("buzzer gpio not ready");
        return -ENODEV;
    }
    int ret = gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);
    if (ret < 0) {
        LOG_ERR("buzzer configure failed: %d", ret);
        return ret;
    }

    k_timer_init(&tone_timer, tone_tick, NULL);
    k_work_init_delayable(&note_work, note_handler);
    note_idx = 0;
    k_work_schedule(&note_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(startup_beep_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
