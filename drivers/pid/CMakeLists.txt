# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_PID pid_mit.c)
zephyr_library_sources_ifdef(CONFIG_PID pid_single.c)
