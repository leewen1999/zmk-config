# Add sources to ZMK's "app" target so ZMK headers (zmk/ble.h etc.) are on the include path
target_sources_ifdef(CONFIG_SETUP_MODE app PRIVATE src/setup_mode.c)
