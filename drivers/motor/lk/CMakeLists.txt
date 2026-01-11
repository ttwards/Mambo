zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MOTOR_LK motor_lk.c)
