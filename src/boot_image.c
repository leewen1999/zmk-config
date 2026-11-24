/*
 * Boot image: shows a 160 x 68 picture on the display (nice!view) for
 * CONFIG_SWANBOARD_DISPLAY_BOOT_IMAGE_MS milliseconds after boot / wake-up
 * (waking up from deep sleep / soft off is a reboot).
 *
 * The picture is src/boot_image_data.c, made by tools/make_boot_image.py.
 *
 * LVGL must only be used from ZMK's display work queue, so both steps (show, hide) are
 * scheduled on zmk_display_work_q(). The picture is put on LVGL's "top layer", above
 * the status screen, and deleted again afterwards.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <lvgl.h>
#include <zmk/display.h>

LOG_MODULE_REGISTER(boot_image, CONFIG_ZMK_LOG_LEVEL);

extern const lv_img_dsc_t swanboard_boot_image;

#define SHOW_RETRY_MS 100
#define SHOW_RETRY_MAX 100 /* 10 s */

static struct k_work_delayable show_work;
static struct k_work_delayable hide_work;
static lv_obj_t *boot_img;
static int show_tries;

static void hide_handler(struct k_work *work) {
    if (boot_img != NULL) {
        lv_obj_del(boot_img);
        boot_img = NULL;
    }
    lv_obj_t *screen = lv_scr_act();
    if (screen != NULL) {
        lv_obj_invalidate(screen); /* redraw the status screen */
    }
}

static void show_handler(struct k_work *work) {
    if (lv_disp_get_default() == NULL) {
        /* LVGL / display not ready yet */
        if (++show_tries < SHOW_RETRY_MAX) {
            k_work_schedule_for_queue(zmk_display_work_q(), &show_work, K_MSEC(SHOW_RETRY_MS));
        }
        return;
    }

    boot_img = lv_img_create(lv_layer_top());
    lv_img_set_src(boot_img, &swanboard_boot_image);
    lv_obj_align(boot_img, LV_ALIGN_TOP_LEFT, 0, 0);

    k_work_schedule_for_queue(zmk_display_work_q(), &hide_work,
                              K_MSEC(CONFIG_SWANBOARD_DISPLAY_BOOT_IMAGE_MS));
}

static int boot_image_init(void) {
    k_work_init_delayable(&show_work, show_handler);
    k_work_init_delayable(&hide_work, hide_handler);
    k_work_schedule_for_queue(zmk_display_work_q(), &show_work, K_NO_WAIT);
    return 0;
}

SYS_INIT(boot_image_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
