# Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

set(USB_BULK_INTERFACE_SOURCES usb_bulk.c)

zephyr_library_sources(${USB_BULK_INTERFACE_SOURCES})
