/*
 * nice!view tweaks applied on top of ZMK's built-in status screen.
 *
 *  - CONFIG_TWELVEPAD_DISPLAY_BLE_SYMBOL: ZMK's built-in output widget draws the
 *    LVGL "Wi-Fi" glyph for Bluetooth. A render-start hook of the LVGL display driver
 *    replaces it with the LVGL Bluetooth glyph right BEFORE every redraw, so there is
 *    no timer and no frame with the Wi-Fi glyph (boot, profile switch, connect ...).
 *    LVGL calls render_start_cb in the display thread at the start of every refresh.
 *    The edit is done in place (both glyphs are 3 bytes in UTF-8), because
 *    lv_label_set_text() must not be called while rendering.
 *
 *  - CONFIG_TWELVEPAD_DISPLAY_LAYER_SYMBOL: the layer widget draws the LVGL keyboard glyph
 *    in front of the layer name. For the layers listed in layer_symbols[] below, the
 *    glyph is replaced (e.g. "Setup" -> settings glyph, "Sleep" -> power glyph). Same
 *    render-start hook, same in-place edit (all glyphs are 3 bytes in UTF-8).
 *
 *  - CONFIG_TWELVEPAD_DISPLAY_ROTATE_180 (experimental): turn the picture by 180
 *    degrees (LVGL software rotation), applied once from a one-shot LVGL timer.
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include <string.h>

#include <lvgl.h>

LOG_MODULE_REGISTER(display_tweaks, CONFIG_ZMK_LOG_LEVEL);

#define HOOK_RETRY_MS 100
#define HOOK_RETRY_MAX 100 /* 10 s */

#if defined(CONFIG_TWELVEPAD_DISPLAY_BLE_SYMBOL) || defined(CONFIG_TWELVEPAD_DISPLAY_LAYER_SYMBOL)

static struct k_work_delayable hook_work;
static int hook_tries;

#ifdef CONFIG_TWELVEPAD_DISPLAY_LAYER_SYMBOL
/* Layer name (the keymap "display-name") -> glyph shown in front of it.
 * Layers that are not listed keep the keyboard glyph. Keep the names in sync with
 * the display-name properties in twelvepad.keymap. */
static const struct {
    const char *name;
    const char *symbol;
} layer_symbols[] = {
    {"Setup", LV_SYMBOL_SETTINGS},
    {"Sleep", LV_SYMBOL_POWER},
};
#endif

/* Edit the texts of all labels of the screen in place (before they are drawn). */
static void patch_labels(lv_obj_t *obj) {
    if (lv_obj_check_type(obj, &lv_label_class)) {
        char *text = lv_label_get_text(obj);

        if (text != NULL) {
#ifdef CONFIG_TWELVEPAD_DISPLAY_BLE_SYMBOL
            /* Wi-Fi glyph -> Bluetooth glyph */
            char *pos = strstr(text, LV_SYMBOL_WIFI);

            if (pos != NULL) {
                memcpy(pos, LV_SYMBOL_BLUETOOTH, strlen(LV_SYMBOL_BLUETOOTH));
            }
#endif
#ifdef CONFIG_TWELVEPAD_DISPLAY_LAYER_SYMBOL
            /* "<keyboard glyph>[ ]<layer name>" -> "<other glyph>[ ]<layer name>" */
            size_t glyph_len = strlen(LV_SYMBOL_KEYBOARD);

            if (strncmp(text, LV_SYMBOL_KEYBOARD, glyph_len) == 0) {
                const char *name = text + glyph_len;

                while (*name == ' ') {
                    name++;
                }
                for (size_t i = 0; i < ARRAY_SIZE(layer_symbols); i++) {
                    if (strcmp(name, layer_symbols[i].name) == 0) {
                        memcpy(text, layer_symbols[i].symbol, glyph_len);
                        break;
                    }
                }
            }
#endif
        }
    }

    uint32_t count = lv_obj_get_child_cnt(obj);
    for (uint32_t i = 0; i < count; i++) {
        patch_labels(lv_obj_get_child(obj, i));
    }
}

static void render_start(lv_disp_drv_t *drv) {
    lv_obj_t *screen = lv_scr_act();

    if (screen != NULL) {
        patch_labels(screen);
    }
}

static bool install_hook(void) {
    lv_disp_t *disp = lv_disp_get_default();

    if (disp == NULL || disp->driver == NULL) {
        return false;
    }
    disp->driver->render_start_cb = render_start;

    /* Redraw once so a frame that was already drawn gets the new symbol. */
    lv_obj_t *screen = lv_scr_act();
    if (screen != NULL) {
        lv_obj_invalidate(screen);
    }
    return true;
}

/* One-shot retries, only until the display is registered. */
static void hook_handler(struct k_work *work) {
    if (!install_hook() && ++hook_tries < HOOK_RETRY_MAX) {
        k_work_schedule(&hook_work, K_MSEC(HOOK_RETRY_MS));
    }
}

#endif /* BLE_SYMBOL || LAYER_SYMBOL */

#ifdef CONFIG_TWELVEPAD_DISPLAY_ROTATE_180
static struct k_work_delayable rotate_work;

static void rotate_cb(lv_timer_t *timer) {
    lv_disp_t *disp = lv_disp_get_default();

    if (disp != NULL) {
        disp->driver->sw_rotate = 1;
        lv_disp_set_rotation(disp, LV_DISP_ROT_180);
        lv_timer_del(timer); /* one-shot */
    }
}

static void rotate_handler(struct k_work *work) {
    lv_timer_create(rotate_cb, 250, NULL);
}
#endif /* CONFIG_TWELVEPAD_DISPLAY_ROTATE_180 */

static int display_tweaks_init(void) {
#if defined(CONFIG_TWELVEPAD_DISPLAY_BLE_SYMBOL) || defined(CONFIG_TWELVEPAD_DISPLAY_LAYER_SYMBOL)
    k_work_init_delayable(&hook_work, hook_handler);
    if (!install_hook()) {
        k_work_schedule(&hook_work, K_MSEC(HOOK_RETRY_MS));
    }
#endif
#ifdef CONFIG_TWELVEPAD_DISPLAY_ROTATE_180
    k_work_init_delayable(&rotate_work, rotate_handler);
    k_work_schedule(&rotate_work, K_MSEC(1000));
#endif
    return 0;
}

SYS_INIT(display_tweaks_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
