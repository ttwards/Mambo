# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

set(CONNECT_SOURCES uart_trans.c uart_async_adapter.c)

zephyr_library_sources(${CONNECT_SOURCES})
