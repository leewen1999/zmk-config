# Add sources to ZMK's "app" target so ZMK headers (zmk/ble.h etc.) are on the include path
target_sources_ifdef(CONFIG_SETUP_MODE app PRIVATE src/setup_mode.c)
target_sources_ifdef(CONFIG_SWANBOARD_DISPLAY_TWEAKS app PRIVATE src/display_tweaks.c)
target_sources_ifdef(CONFIG_SWANBOARD_BATTERY_LINEAR app PRIVATE src/battery_linear.c)
target_sources_ifdef(CONFIG_SWANBOARD_STARTUP_BEEP app PRIVATE src/startup_beep.c)
