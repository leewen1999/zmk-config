# Add sources to ZMK's "app" target so ZMK headers (zmk/ble.h etc.) are on the include path
target_sources_ifdef(CONFIG_SETUP_MODE app PRIVATE src/setup_mode.c)
target_sources_ifdef(CONFIG_TWELVEPAD_DISPLAY_TWEAKS app PRIVATE src/display_tweaks.c)
target_sources_ifdef(CONFIG_TWELVEPAD_BATTERY_CR2032 app PRIVATE src/battery_cr2032.c)
