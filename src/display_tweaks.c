/*
 * nice!view tweaks applied on top of ZMK's built-in status screen.
 *
 *  - CONFIG_TWELVEPAD_DISPLAY_ROTATE_180: turn the picture by 180 degrees
 *    (LVGL software rotation).
 *  - CONFIG_TWELVEPAD_DISPLAY_BLE_SYMBOL: ZMK's built-in output widget draws the
 *    LVGL "Wi-Fi" glyph for Bluetooth; replace it with the LVGL Bluetooth glyph.
 *
 * Both are done from an LVGL timer, so they run in the display thread (where all
 * LVGL calls belong). The timer is created ~1 s after boot, when the screen exists.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>

#include <lvgl.h>

LOG_MODULE_REGISTER(display_tweaks, CONFIG_ZMK_LOG_LEVEL);

#define START_DELAY_MS 1000
#define TWEAK_PERIOD_MS 250

static struct k_work_delayable start_work;

#ifdef CONFIG_TWELVEPAD_DISPLAY_ROTATE_180
static bool rotated;
#endif

#ifdef CONFIG_TWELVEPAD_DISPLAY_BLE_SYMBOL
/* LV_SYMBOL_WIFI and LV_SYMBOL_BLUETOOTH are both 3 bytes long in UTF-8. */
static void patch_labels(lv_obj_t *obj) {
    if (lv_obj_check_type(obj, &lv_label_class)) {
        const char *text = lv_label_get_text(obj);
        const char *pos = (text != NULL) ? strstr(text, LV_SYMBOL_WIFI) : NULL;

        if (pos != NULL && strlen(text) < 64) {
            char buf[64];

            strcpy(buf, text);
            memcpy(buf + (pos - text), LV_SYMBOL_BLUETOOTH, strlen(LV_SYMBOL_BLUETOOTH));
            lv_label_set_text(obj, buf);
        }
    }

    uint32_t count = lv_obj_get_child_cnt(obj);
    for (uint32_t i = 0; i < count; i++) {
        patch_labels(lv_obj_get_child(obj, i));
    }
}
#endif

static void tweak_cb(lv_timer_t *timer) {
#ifdef CONFIG_TWELVEPAD_DISPLAY_ROTATE_180
    if (!rotated) {
        lv_disp_t *disp = lv_disp_get_default();

        if (disp != NULL) {
            disp->driver->sw_rotate = 1;
            lv_disp_set_rotation(disp, LV_DISP_ROT_180);
            rotated = true;
        }
    }
#endif
#ifdef CONFIG_TWELVEPAD_DISPLAY_BLE_SYMBOL
    patch_labels(lv_scr_act());
#endif
}

static void start_handler(struct k_work *work) {
    lv_timer_create(tweak_cb, TWEAK_PERIOD_MS, NULL);
}

static int display_tweaks_init(void) {
    k_work_init_delayable(&start_work, start_handler);
    k_work_schedule(&start_work, K_MSEC(START_DELAY_MS));
    return 0;
}

SYS_INIT(display_tweaks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
