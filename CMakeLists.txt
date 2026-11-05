# Add sources to ZMK's "app" target so ZMK headers (zmk/ble.h etc.) are on the include path
target_sources_ifdef(CONFIG_LED_INDICATOR app PRIVATE src/led_indicator.c)
