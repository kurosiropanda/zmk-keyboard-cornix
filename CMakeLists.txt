# Compile in-repo ZMK feature sources as part of the ZMK `app` target so they
# inherit the ZMK application include paths (zmk/event_manager.h, etc.).
# Enabled via Kconfig; no effect on builds where the option is off.

target_sources_ifdef(CONFIG_CORNIX_STATUS_LED app PRIVATE boards/jzf/cornix/status_led.c)
