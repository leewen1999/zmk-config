/*
 * CR2032 battery sensor wrapper.
 *
 * ZMK reads the battery through the sensor named by /chosen/zmk,battery and asks it for
 * SENSOR_CHAN_GAUGE_STATE_OF_CHARGE. The stock drivers convert the voltage with a
 * Li-ion formula (3.45 V = 0 %, 4.2 V = 100 %), which is useless for a CR2032.
 *
 * This wrapper is chosen instead (see twelvepad.overlay): it takes the voltage from the
 * original ZMK battery sensor ("source") and maps it linearly:
 *     full-mv  (default 3200 mV) -> 100 %
 *     empty-mv (default 2700 mV) ->   0 %
 * offset-mv (default 0) is added to the measured voltage first (calibration).
 * Everything that uses the battery level (display, BLE battery service, low battery
 * LED) gets the new value.
 */

#define DT_DRV_COMPAT twelvepad_battery_cr2032

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery_cr2032, CONFIG_SENSOR_LOG_LEVEL);

struct batt_cfg {
    const struct device *source;
    int32_t full_mv;
    int32_t empty_mv;
    int32_t offset_mv;
};

struct batt_data {
    int32_t mv;
    uint8_t soc;
};

static uint8_t mv_to_pct(int32_t mv, int32_t full_mv, int32_t empty_mv) {
    if (mv >= full_mv) {
        return 100;
    }
    if (mv <= empty_mv) {
        return 0;
    }
    return (uint8_t)(((mv - empty_mv) * 100) / (full_mv - empty_mv));
}

static int batt_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    const struct batt_cfg *cfg = dev->config;
    struct batt_data *data = dev->data;
    struct sensor_value val;
    int ret;

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_GAUGE_VOLTAGE &&
        chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE) {
        return -ENOTSUP;
    }

    ret = sensor_sample_fetch_chan(cfg->source, SENSOR_CHAN_GAUGE_VOLTAGE);
    if (ret < 0) {
        LOG_ERR("source fetch failed: %d", ret);
        return ret;
    }
    ret = sensor_channel_get(cfg->source, SENSOR_CHAN_GAUGE_VOLTAGE, &val);
    if (ret < 0) {
        LOG_ERR("source voltage read failed: %d", ret);
        return ret;
    }

    data->mv = val.val1 * 1000 + val.val2 / 1000 + cfg->offset_mv;
    data->soc = mv_to_pct(data->mv, cfg->full_mv, cfg->empty_mv);
    LOG_DBG("battery %d mV -> %u %%", data->mv, data->soc);
    return 0;
}

static int batt_channel_get(const struct device *dev, enum sensor_channel chan,
                            struct sensor_value *val) {
    const struct batt_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val->val1 = data->mv / 1000;
        val->val2 = (data->mv % 1000) * 1000;
        return 0;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val->val1 = data->soc;
        val->val2 = 0;
        return 0;
    default:
        return -ENOTSUP;
    }
}

static int batt_init(const struct device *dev) {
    const struct batt_cfg *cfg = dev->config;

    if (!device_is_ready(cfg->source)) {
        LOG_ERR("source battery sensor not ready");
        return -ENODEV;
    }
    if (cfg->full_mv <= cfg->empty_mv) {
        LOG_ERR("full-mv must be larger than empty-mv");
        return -EINVAL;
    }
    return 0;
}

static const struct sensor_driver_api batt_api = {
    .sample_fetch = batt_sample_fetch,
    .channel_get = batt_channel_get,
};

#define BATT_INST(n)                                                                               \
    static struct batt_data batt_data_##n;                                                         \
    static const struct batt_cfg batt_cfg_##n = {                                                  \
        .source = DEVICE_DT_GET(DT_INST_PHANDLE(n, source)),                                       \
        .full_mv = DT_INST_PROP(n, full_mv),                                                       \
        .empty_mv = DT_INST_PROP(n, empty_mv),                                                     \
        .offset_mv = DT_INST_PROP(n, offset_mv),                                                   \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, batt_init, NULL, &batt_data_##n, &batt_cfg_##n, POST_KERNEL,          \
                          CONFIG_SENSOR_INIT_PRIORITY, &batt_api);

DT_INST_FOREACH_STATUS_OKAY(BATT_INST)
