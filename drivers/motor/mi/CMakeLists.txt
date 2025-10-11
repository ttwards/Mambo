zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MOTOR_MI motor_mi.c)
