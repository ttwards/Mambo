/*
 * Copyright (c) 2025 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ARES_UART_H__
#define ARES_UART_H__

#include "zephyr/sys/ring_buffer.h"
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include "ares/interface/ares_interface.h"

int ares_uart_init(struct AresInterface *interface);
int ares_uart_send(struct AresInterface *interface, struct net_buf *buf);
struct net_buf *ares_uart_interface_alloc_buf(struct AresInterface *interface);

void ares_uart_init_dev(struct AresInterface *interface, const struct device *uart_dev);

#define ARES_UART_PROCESSING_THREAD_STACK_SIZE CONFIG_ARES_UART_THREAD_STACK_SIZE
#define ARES_UART_TX_THREAD_STACK_SIZE CONFIG_ARES_UART_THREAD_STACK_SIZE

#define ARES_UART_BLOCK_SIZE    72
#define MAX_FRAME_PAYLOAD_SIZE  ARES_UART_BLOCK_SIZE
#define ARES_UART_TX_QUEUE_SIZE 10


struct AresUartInterface {
	struct AresInterface *interface;

	atomic_t state;

	struct ring_buf uart_rb;
	uint8_t uart_rb_buf[4 * ARES_UART_BLOCK_SIZE];

	struct device *uart_dev;

	struct k_sem sem;

	struct k_thread thread;
	k_thread_stack_t thread_stack[ARES_UART_PROCESSING_THREAD_STACK_SIZE];

	struct k_msgq tx_msgq;                 // 发送消息队列
    char __aligned(4) tx_msgq_buffer[sizeof(struct net_buf *) * ARES_UART_TX_QUEUE_SIZE];
    struct k_sem tx_sem;                   // 用于发送流控制的信号量
    struct net_buf *current_tx_buf;        // 指向当前正在发送的buf
	struct k_thread tx_thread;             // 发送线程的句柄
    k_thread_stack_t tx_thread_stack[ARES_UART_TX_THREAD_STACK_SIZE]; // 发送线程的栈
};

#define ARES_UART_INTERFACE_DEFINE(Interface_name)                                                 \
	struct AresInterfaceAPI ares_uart_interface_api = {                                        \
		.init = ares_uart_init,                                                            \
		.send = ares_uart_send,                                                           \
		.alloc_buf = ares_uart_interface_alloc_buf,                                             \
	};                                                                                         \
	struct AresUartInterface Internal_##Interface_name = {NULL};                               \
	struct AresInterface Interface_name = {                                                    \
		.name = #Interface_name,                                                           \
		.api = &ares_uart_interface_api,                                                   \
		.protocol = NULL,                                                                  \
		.priv_data = &Internal_##Interface_name,                                             \
	};

#endif // ARES_UART_H__