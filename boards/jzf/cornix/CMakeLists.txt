zephyr_library()
zephyr_library_sources(pinmux.c)
zephyr_library_sources_ifdef(CONFIG_CORNIX_STATUS_LED status_led.c)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)