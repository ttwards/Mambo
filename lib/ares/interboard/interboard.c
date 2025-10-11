#include "interboard.h"
#include "zephyr/logging/log.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/sys/crc.h>

LOG_MODULE_REGISTER(interboard, LOG_LEVEL_DBG);

K_MSGQ_DEFINE(interboardrxcan_msgq, sizeof(Frame_Data), MSG_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(interboardrximu_msgq, sizeof(Frame_Data), MSG_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(interboardtx_msgq, sizeof(Frame_Data), MSG_QUEUE_SIZE, 4);

K_THREAD_STACK_DEFINE(interboard_work_queue_stack, SPI_SEND_STACK_SIZE);
K_WORK_DEFINE(interboard_rx_data_handle, interboard_rx_data_handler);
K_WORK_DEFINE(interboard_tx_data_handle, interboard_tx_data_handler);
K_TIMER_DEFINE(interboard_timer, interboard_tx_isr_handler, NULL);

struct spi_config spi_cfg = {
	.frequency = 10500000,
#ifdef CONFIG_MASTER_BOARD
	.operation = (spi_operation_t)(SPI_OP_MODE_MASTER | SPI_LINES_DUAL | SPI_WORD_SET(8) |
				       SPI_TRANSFER_LSB | SPI_MODE_CPOL | SPI_MODE_CPHA),

#endif
#ifdef CONFIG_SLAVE_BOARD
	.operation = (spi_operation_t)(SPI_OP_MODE_SLAVE | SPI_LINES_DUAL | SPI_WORD_SET(8) |
				       SPI_TRANSFER_LSB | SPI_MODE_CPOL | SPI_MODE_CPHA),
#endif
	.slave = 0,
};

struct k_work_q interboard_work_queue;

const struct device *spi;

uint16_t calculate_crc16(const uint8_t *data, size_t length)
{
	return crc16(0x8005, 0xFFFF, data, length);
}
uint8_t msgcounter = 0;
bool received = true;

void build_txbuf(uint8_t *tx_buffer, InterboardMsgType msg_type, uint8_t *data, bool is_retransmit)
{
	uint8_t is_master = 1;
#ifdef CONFIG_SLAVE_BOARD
	is_master = 0;
#endif
	// 直接操作 buffer
	uint8_t *ptr = tx_buffer;

	// 同步头 (0x5A)
	*ptr = 0x5A;
	ptr++;

	// 构造控制字段 (is_master << 7) | (is_retransmit << 6)
	*ptr = (is_master << 7) | (is_retransmit << 6);
	ptr++;

	// 消息类型
	*ptr = msg_type;
	ptr++;
	if (received) {
		*ptr = (msgcounter << 4) | RECEIVE_ACK;
	} else {
		*ptr = msgcounter << 4;
	}
	ptr++;
	// 填充数据区
	memcpy(ptr, data, 16); // 假设 data 长度为 16
	ptr += 16;

	// 计算 CRC16
	size_t length = datalen + 4; // 当前指针位置即为数据长度（不包含 CRC16 和帧尾）
	uint16_t crc = calculate_crc16(tx_buffer, length);
	*(uint16_t *)ptr = crc; // CRC16 放入缓冲区
	ptr += 2;

	// 设置帧尾
	*(uint16_t *)ptr = 0x0D0A; // 帧尾 (0x0D0A)
}
void process_rxbuf(uint8_t *rx_buffer, InterboardMsgType msg_type, uint8_t *data,
		   bool is_retransmit)
{
	uint8_t *ptr = rx_buffer;

	// 同步头 (0x5A)
	if (*ptr != 0x5A) {
		LOG_ERR("Error: Invalid sync byte\n");
		return;
	}
	ptr++;

	// 控制字段 (is_master << 7) | (is_retransmit << 6)
	uint8_t is_master = *ptr >> 7;
	if (is_master) {
		// do something
	}
	is_retransmit = (bool)((*ptr >> 6) & 0x01);
	ptr++;

	// 消息类型
	msg_type = *ptr;
	ptr++;
	if ((*ptr & 0x0F) != RECEIVE_ACK) {
		if (msgcounter != 1) {
			// received = false;
			LOG_ERR("donot received");
		}
	}
	LOG_INF("msgcounter: %d", *ptr >> 4);
	// 应答

	ptr++;
	uint8_t *data_local = ptr;
	ptr += 16;

	size_t length = datalen + 4; // 当前 ptr 位置即为数据部分长度（不包含 CRC16 和帧尾）
	uint16_t received_crc = *(uint16_t *)ptr;
	uint16_t calculated_crc = calculate_crc16(rx_buffer, length);
	if (received_crc != calculated_crc) {
		LOG_ERR("Error: CRC mismatch\n");
		return;
	}
	ptr += 2;
	if (*(uint16_t *)ptr != 0x0D0A) {
		LOG_ERR("Error: Invalid end mark\n");
		return;
	}

	memcpy(data, data_local, datalen);
}
Frame_Data tx_data_out;                   // 定时发送的数据
Frame_Data *tx_data_out_p = &tx_data_out; // 定时发送的数据来源指针
static uint8_t __aligned(8) tx_buffer_local[SPI_BUF_SIZE] = {0};
static uint8_t __aligned(8) rx_buffer_local[SPI_BUF_SIZE] = {0};
struct spi_buf tx_buf = {
	.buf = &tx_buffer_local, // Initialize with tx_buffer_local
	.len = SPI_BUF_SIZE,
};
struct spi_buf_set tx_bufs = {
	.buffers = &tx_buf,
	.count = 1,
};
struct spi_buf rx_buf = {
	.buf = &rx_buffer_local, // Initialize with NULL
	.len = SPI_BUF_SIZE,
};
struct spi_buf_set rx_bufs = {
	.buffers = &rx_buf,
	.count = 1,
};

void interboard_rx_data_handler(struct k_work *work)
{
	Frame_Data rx_data = {0};
	bool is_retransmit = false;
	process_rxbuf(rx_buffer_local, rx_data.msg_type, rx_data.data, is_retransmit);
	switch (rx_data.msg_type) {
	case INTERBOARD_MSG_CAN:
		k_msgq_put(&interboardrxcan_msgq, &rx_data, K_NO_WAIT);
		break;
	case INTERBOARD_MSG_IMU:
		k_msgq_put(&interboardrximu_msgq, &rx_data, K_NO_WAIT);
		break;
	}
}

void interboardrx_callback(const struct device *spi_dev, int error_code, void *data)
{
	if (error_code == 0) {
		k_work_submit_to_queue(&interboard_work_queue, &interboard_rx_data_handle);
#ifdef CONFIG_SLAVE_BOARD
		k_work_submit_to_queue(&interboard_work_queue, &interboard_tx_data_handle);
#endif
	} else if (error_code != 0) {
		LOG_ERR(" transfer met an error\n");
		// 释放spi
		spi_release(spi_dev, &spi_cfg);
	}
}

void interboard_tx_data_handler(struct k_work *work)
{
	if (k_msgq_get(&interboardtx_msgq, tx_data_out_p, K_NO_WAIT) == 0) {
		build_txbuf(tx_buffer_local, tx_data_out_p->msg_type, tx_data_out_p->data, false);
		msgcounter++;
	}
	int error = spi_transceive_cb(spi, &spi_cfg, &tx_bufs, &rx_bufs, interboardrx_callback,
				      rx_buf.buf);
	if (error != 0) {
		LOG_ERR("spi_errorcode: %d", error);
	}
}

void interboard_tx_isr_handler(struct k_timer *dummy)
{
#ifdef CONFIG_MASTER_BOARD
	k_work_submit_to_queue(&interboard_work_queue, &interboard_tx_data_handle);
#endif
}

// 初始化timer，设置定时器回调函数，设置定时器数据（要提交的工作）
void interboard_init(struct device *spi_dev)
{
	spi = spi_dev;
	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready\n");
		return;
	}
	k_work_queue_init(&interboard_work_queue);
	k_work_queue_start(&interboard_work_queue, interboard_work_queue_stack, SPI_SEND_STACK_SIZE,
			   SPI_SEND_PRIORITY, NULL);
	k_timer_start(&interboard_timer, K_MSEC(1), K_MSEC(1));
#ifdef CONFIG_SLAVE_BOARD
	k_sleep(K_MSEC(1000));
	k_work_submit_to_queue(&interboard_work_queue, &interboard_tx_data_handle);
#endif
}

int interboard_transceive(Frame_Data *tx_data, Frame_Data *rx_data)
{
	k_msgq_put(&interboardtx_msgq, tx_data, K_NO_WAIT);
	switch (tx_data->msg_type) {
	case INTERBOARD_MSG_CAN:
		k_msgq_get(&interboardrxcan_msgq, rx_data, K_FOREVER);
		return 0;
	case INTERBOARD_MSG_IMU:
		k_msgq_get(&interboardrximu_msgq, rx_data, K_FOREVER);
		return 0;
	}
	return 0;
}
