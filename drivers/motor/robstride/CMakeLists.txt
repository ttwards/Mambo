zephyr_library()
zephyr_library_sources_ifdef(CONFIG_MOTOR_RS motor_rs.c)
