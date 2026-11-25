/*
 * Boot image: shows a 160 x 68 picture on the display (nice!view) for
 * CONFIG_SWANBOARD_DISPLAY_BOOT_IMAGE_MS milliseconds after boot / wake-up
 * (waking up from deep sleep / soft off is a reboot).
 *
 * The picture is src/boot_image_data.c, made by tools/make_boot_image.py.
 *
 * LVGL must only be used from ZMK's display work queue, so the steps (show, hide) are
 * scheduled on zmk_display_work_q(). The picture is put on LVGL's "top layer", above
 * the status screen, and deleted again afterwards.
 *
 * The first step is started from the SYSTEM work queue and retried until the display work
 * queue accepts work: that queue is only started by zmk_display_init(), which runs after
 * this file's SYS_INIT, and k_work_schedule_for_queue() refuses work (-ENODEV) for a queue
 * that is not started yet.
 *
 * The countdown starts when the picture has really been DRAWN (LV_EVENT_DRAW_MAIN_END),
 * not when the object is created: the display needs some time after boot, and a fixed
 * time from creation could run out before anything is visible. If the picture is never
 * drawn, it is removed after FALLBACK_MS anyway.
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
#define FALLBACK_MS 10000
#define KICK_RETRY_MS 100
#define KICK_RETRY_MAX 100 /* 10 s */

static struct k_work_delayable kick_work; /* runs on the system work queue */
static struct k_work_delayable show_work; /* runs on the display work queue */
static struct k_work_delayable hide_work; /* runs on the display work queue */
static lv_obj_t *boot_img;
static int show_tries;
static int kick_tries;
static bool hide_started;

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

/* Called (in the display thread) every time the picture has been drawn. */
static void drawn_cb(lv_event_t *e) {
    if (!hide_started) {
        hide_started = true;
        LOG_INF("boot picture drawn");
        k_work_reschedule_for_queue(zmk_display_work_q(), &hide_work,
                                    K_MSEC(CONFIG_SWANBOARD_DISPLAY_BOOT_IMAGE_MS));
    }
}

static void show_handler(struct k_work *work) {
    if (lv_disp_get_default() == NULL) {
        /* LVGL / display not ready yet */
        if (++show_tries < SHOW_RETRY_MAX) {
            k_work_schedule_for_queue(zmk_display_work_q(), &show_work, K_MSEC(SHOW_RETRY_MS));
        } else {
            LOG_ERR("display never became ready");
        }
        return;
    }

    boot_img = lv_img_create(lv_layer_top());
    if (boot_img == NULL) {
        LOG_ERR("lv_img_create failed (LVGL memory pool too small?)");
        return;
    }
    lv_img_set_src(boot_img, &swanboard_boot_image);
    lv_obj_align(boot_img, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_add_event_cb(boot_img, drawn_cb, LV_EVENT_DRAW_MAIN_END, NULL);
    LOG_INF("boot picture created");

    /* Safety net: remove the picture even if it is never drawn. */
    k_work_schedule_for_queue(zmk_display_work_q(), &hide_work, K_MSEC(FALLBACK_MS));
}

/* System work queue: hand the first step over to the display work queue as soon as that
 * queue has been started (k_work_schedule_for_queue() returns a negative errno before). */
static void kick_handler(struct k_work *work) {
    int ret = k_work_schedule_for_queue(zmk_display_work_q(), &show_work, K_NO_WAIT);

    if (ret < 0) {
        if (++kick_tries < KICK_RETRY_MAX) {
            k_work_schedule(&kick_work, K_MSEC(KICK_RETRY_MS));
        } else {
            LOG_ERR("display work queue never accepted work: %d", ret);
        }
        return;
    }
    LOG_INF("boot picture step scheduled on the display queue (after %d retries)", kick_tries);
}

static int boot_image_init(void) {
    k_work_init_delayable(&kick_work, kick_handler);
    k_work_init_delayable(&show_work, show_handler);
    k_work_init_delayable(&hide_work, hide_handler);
    k_work_schedule(&kick_work, K_MSEC(KICK_RETRY_MS));
    return 0;
}

SYS_INIT(boot_image_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
