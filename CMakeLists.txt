# Add sources to ZMK's "app" target so ZMK headers (zmk/ble.h etc.) are on the include path
target_sources_ifdef(CONFIG_BLE_STATUS_LED app PRIVATE src/ble_status_led.c)
