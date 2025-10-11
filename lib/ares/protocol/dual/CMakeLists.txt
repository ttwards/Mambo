# Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

set(DUAL_PROTOCOL_SOURCES dual_protocol.c)

zephyr_library_sources(${DUAL_PROTOCOL_SOURCES})
