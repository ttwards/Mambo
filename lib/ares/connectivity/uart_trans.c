#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/toolchain.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/crc.h>
#include <zephyr/usb/usb_device.h>
#include <ares/connectivity/uart_trans.h>
#include <sys/types.h>
#include "uart_async_adapter.h"
#include "zephyr/sys/ring_buffer.h"

LOG_MODULE_REGISTER(uart_trans, CONFIG_LOG_DEFAULT_LEVEL);

// usb async adapter use millisecond as timeout
// but uart use microsecond as timeout
#if (CONFIG_USB_UART_ASYNC_ADAPTER)
#define RX_INACTIVE_TIMEOUT 15
#else
#define RX_INACTIVE_TIMEOUT 15
#endif

#define CDC_NODE DT_CHOSEN(ares_usb)

struct device *uart_trans_dev = (struct device *)DEVICE_DT_GET(CDC_NODE);

// serial buffer pool
#define FUNC_REPL_SIZE   12
#define BUF_SIZE         270
#define SYNC_RX_BUF_SIZE (BUF_SIZE)
#define SYNC_TX_BUF_SIZE (BUF_SIZE)

#define MAX_PACK_COUNT 8
#define MAX_FUNC_COUNT MAX_PACK_COUNT

RING_BUF_DECLARE(uart_rx_rb, 280);
RING_BUF_DECLARE(incomplete_rb, 280);

uint16_t incomplete_target_tail = 0;

K_MEM_SLAB_DEFINE(uart_rx_slab, SYNC_RX_BUF_SIZE, 12, 4);
K_MEM_SLAB_DEFINE(func_ret_slab, (FUNC_REPL_SIZE + 4), 8, 4);

uint8_t func_tx_bckup_cnt = 0;
K_FIFO_DEFINE(func_tx_bckup_fifo);

#define HEART_BEAT_DELAY 2

uint8_t error_frame[8];

static void uart_trans_heart_beat(struct k_timer *timer);
K_TIMER_DEFINE(heart_beat_timer, uart_trans_heart_beat, NULL);

bool online = false;
uint32_t last_heart_beat = 0;
uint32_t last_receive = 0;

bool rx_throttle = false;
static bool uart_initialized = false;

static int8_t func_cnt = 0;
static func_mapping_t func_table[MAX_PACK_COUNT];

static int8_t sync_pack_cnt = 0;
static uart_sync_pack_t sync_pack[MAX_PACK_COUNT];

static void error_handle(uint16_t req_id, uint16_t error)
{
	GET_8BITS(error_frame, ERROR_ID_IDX) = req_id;
	GET_16BITS(error_frame, ERROR_CODE_IDX) = error;
	if (error != HEART_BEAT) {
		// LOG_ERR("Error code: %x, Request ID: %x", error, req_id);
	} else {
		last_heart_beat = k_cycle_get_32();
	}
	uart_tx(uart_trans_dev, error_frame, 8, 10);
}

static bool proceed_tx_bck(uint8_t ID)
{
	// 创建一个临时队列
	struct k_fifo temp_fifo;
	k_fifo_init(&temp_fifo);
	bool found = false;

	// 临时存储所有数据项
	uint8_t *item;
	while ((item = k_fifo_get(&func_tx_bckup_fifo, K_NO_WAIT)) != NULL) {
		if (GET_8BITS(item, REPL_REQ_ID_IDX + 4) == ID && !found) {
			// 找到匹配的项目
			uart_tx(uart_trans_dev, (const uint8_t *)(item + 4), FUNC_REPL_SIZE, 20);
			found = true;
			continue;
		} else {
			// 保存到临时队列
			k_fifo_put(&temp_fifo, item);
		}
	}

	// 将所有项目放回原始队列
	while ((item = k_fifo_get(&temp_fifo, K_NO_WAIT)) != NULL) {
		k_fifo_put(&func_tx_bckup_fifo, item);
	}

	return found;
}

static uart_sync_pack_t *find_pack(uint16_t ID)
{
	for (int i = 0; i < sync_pack_cnt; i++) {
		if (ID == sync_pack[i].ID) {
			return &sync_pack[i];
		}
	}
	return NULL;
}

static func_mapping_t *find_func(uint16_t ID)
{
	for (int i = 0; i < func_cnt; i++) {
		if (ID == func_table[i].id) {
			return &func_table[i];
		}
	}
	return NULL;
}

static void uart_trans_heart_beat(struct k_timer *timer)
{
	if (online) {
		if (k_cyc_to_ms_near32(k_cycle_get_32() - last_heart_beat) <= HEART_BEAT_DELAY &&
		    k_cyc_to_ms_near32(k_cycle_get_32() - last_receive) >= HEART_BEAT_DELAY) {
			LOG_ERR("Connection lost for %d ms.",
				k_cyc_to_ms_near32(k_cycle_get_32() - last_receive));
			online = false;
			last_heart_beat = k_cycle_get_32();
		}
	}
	error_handle(HEARTBEAT_ID, HEART_BEAT);
	last_heart_beat = k_cycle_get_32();
}

static void parse_sync(uint8_t data[], uart_sync_pack_t *pack, uint8_t len)
{
	LOG_HEXDUMP_DBG(data, len, "SYNC frame received");
	uint8_t crc8 = 0x00;
	int data_len;
	data_len = len - 8;
	crc8 = crc8_ccitt(0x00, data + 2, data_len + 2);
	if (crc8 != GET_8BITS(data, data_len + SYNC_CRC_OFFSET)) {
		LOG_ERR("CRC mismatch.");
		LOG_HEXDUMP_DBG(data, len, "SYNC frame received");

		error_handle(pack->ID, UNMATCH_CRCx);

		return;
	}
	if (pack->cb != NULL) {
		pack->cb(SYNC_PACK_STATUS_WRITE);
	}
	memcpy(pack->data, data + SYNC_DATA_IDX, data_len);
	return;
}

static void parse_func(uint8_t data[], int8_t len)
{
	uint8_t crc8 = 0x00;
	func_mapping_t *map = find_func(GET_16BITS(data, FUNC_ID_IDX));
	LOG_HEXDUMP_DBG(data, len, "FUNC frame received");
	if (map == NULL) {
		LOG_ERR("Cannot find corresponding ID in func mappings.");
		// error_handle(0, UNKNOWN_FUNC);
		return;
	}
	crc8 = crc8_ccitt(0x00, data + 2, 15);
	if (crc8 != GET_8BITS(data, FUNC_CRC_IDX)) {
		error_handle(GET_16BITS(data, FUNC_REQ_IDX), UNMATCH_CRCx);
		return;
	}
	LOG_DBG("Received Arguments: %x %x %x", GET_32BITS(data, FUNC_ARG1_IDX),
		GET_32BITS(data, FUNC_ARG2_IDX), GET_32BITS(data, FUNC_ARG3_IDX));
	uint32_t ret = map->cb(GET_32BITS(data, FUNC_ARG1_IDX), GET_32BITS(data, FUNC_ARG2_IDX),
			       GET_32BITS(data, FUNC_ARG3_IDX));

	if (func_tx_bckup_cnt == MAX_FUNC_COUNT) {
		uint8_t *rm_frame = k_fifo_get(&func_tx_bckup_fifo, K_NO_WAIT);
		if (rm_frame != NULL) {
			func_tx_bckup_cnt--;
			k_mem_slab_free(&func_ret_slab, &rm_frame);
		} else {
			LOG_ERR("Failed to free memory for FUNC frame.");
			error_handle(GET_16BITS(data, FUNC_REQ_IDX), CLEANED_PACK);
			return;
		}
	}

	uint8_t *repl_frame;
	int err = k_mem_slab_alloc(&func_ret_slab, (void **)&repl_frame, K_NO_WAIT);
	repl_frame += 4;
	if (err != 0) {
		LOG_ERR("Failed to allocate memory for FUNC frame.");
		error_handle(GET_16BITS(data, FUNC_ID_IDX), CLEANED_PACK);
		return;
	}
	GET_16BITS(repl_frame, REPL_HEAD_IDX) = REPL_FRAME_HEAD;
	GET_16BITS(repl_frame, REPL_FUNC_ID_IDX) = GET_16BITS(data, FUNC_ID_IDX);
	GET_32BITS(repl_frame, REPL_RET_IDX) = (uint32_t)ret;
	GET_8BITS(repl_frame, REPL_REQ_ID_IDX) = GET_16BITS(data, FUNC_REQ_IDX);
	GET_16BITS(repl_frame, REPL_TAIL_IDX) = REPL_FRAME_TAIL;
	crc8 = crc8_ccitt(0x00, (uint8_t *)repl_frame + 2, 7);
	GET_8BITS(repl_frame, REPL_CRC_IDX) = crc8;

	k_fifo_put(&func_tx_bckup_fifo, repl_frame - 4);
	if (err != 0) {
		LOG_ERR("Failed to put FUNC frame into FIFO.");
		k_mem_slab_free(&func_ret_slab, (void **)(&repl_frame - 4));
		return;
	}
	func_tx_bckup_cnt++;
	uart_tx(uart_trans_dev, (const uint8_t *)repl_frame, FUNC_REPL_SIZE, 20);
	return;
}

static void parse_error(uint8_t data[], int8_t len)
{
	if (GET_16BITS(data, 6) != ERROR_FRAME_TAIL) {
		LOG_HEXDUMP_DBG(data, len, "Invalid ERROR frame.");
		LOG_ERR("Invalid ERROR frame tail. %x", GET_16BITS(data, 7));
		return;
	}
	uint8_t req_id = GET_8BITS(data, ERROR_ID_IDX);
	if (req_id == HEARTBEAT_ID) {
		return;
	}
	if (!proceed_tx_bck(req_id)) {
		uart_sync_pack_t *pack = find_pack(sys_be16_to_cpu(GET_16BITS(data, 2)));
		if (pack == NULL) {
			LOG_ERR("Cannot find corresponding ID from error "
				"frame.");
			error_handle(req_id, CLEANED_PACK);
			return;
		}
		if (pack->cb != NULL) {
			pack->cb(SYNC_PACK_STATUS_READ);
		}
		uart_trans_sync_flush(pack);
	}
	LOG_INF("Error frame received: id =%d; code=%d", req_id, GET_16BITS(data, ERROR_CODE_IDX));
	return;
}

// 数据处理函数
static void process_received_data()
{
	uint16_t head;
	while (ring_buf_size_get(&uart_rx_rb) >= 8) {
		ring_buf_get(&uart_rx_rb, (uint8_t *)&head, 2);
		if (head == SYNC_FRAME_HEAD) {
			uint16_t id = 0;
			if (ring_buf_get(&uart_rx_rb, (uint8_t *)&id, 2) != 2) {
				LOG_ERR("Failed to get ID from ring buffer.");
				ring_buf_reset(&uart_rx_rb);
				return;
			}
			int16_t pack_len = -1;
			uart_sync_pack_t *pack;
			for (int i = 0; i < sync_pack_cnt; i++) {
				if (sync_pack[i].ID == id) {
					pack_len = sync_pack[i].len + 8;
					pack = &sync_pack[i];
					break;
				}
			}
			if (pack_len == -1) {
				LOG_ERR("Cannot find corresponding ID 0x%x in sync packs.", id);
				// error_handle(0, UNKNOWN_SYNC);
				ring_buf_reset(&uart_rx_rb);
				return;
			}
			uint8_t buf[pack_len];

			GET_16BITS(buf, 0) = SYNC_FRAME_HEAD;
			GET_16BITS(buf, 2) = id;

			int ret = ring_buf_get(&uart_rx_rb, buf + 4, pack_len - 4);
			if (ret < pack_len - 4) {
				if (ring_buf_size_get(&incomplete_rb) > 0) {
					LOG_ERR("Incomplete SYNC frame.");
					ring_buf_reset(&incomplete_rb);
				}
				incomplete_target_tail = SYNC_FRAME_TAIL;
				ring_buf_put(&incomplete_rb, buf, ret + 4);
				return;
			}
			if (GET_16BITS(buf + 4, ret - 2) != SYNC_FRAME_TAIL) {
				LOG_HEXDUMP_DBG(buf, ret + 4, "Invalid SYNC frame tail.");
				// error_handle(id, UNKNOWN_TAIL);
				ring_buf_reset(&uart_rx_rb);
				return;
			}

			parse_sync(buf, pack, pack_len);
			continue;
		} else if (head == FUNC_FRAME_HEAD) {
			uint8_t buf[20];
			GET_16BITS(buf, 0) = FUNC_FRAME_HEAD;
			int ret = ring_buf_get(&uart_rx_rb, buf + 2, 18);
			if (ret < 18) {
				if (!ring_buf_is_empty(&incomplete_rb)) {
					LOG_ERR("Incomplete frame.");
					ring_buf_reset(&incomplete_rb);
				}
				ring_buf_put(&incomplete_rb, buf, ret + 2);
				incomplete_target_tail = FUNC_FRAME_TAIL;
				return;
			}
			parse_func(buf, 20);
			continue;
		} else if (head == ERROR_FRAME_HEAD) {
			uint8_t buf[8];
			GET_16BITS(buf, 0) = ERROR_FRAME_HEAD;
			if (ring_buf_get(&uart_rx_rb, buf + 2, 6) != 6) {
				LOG_ERR("Failed to get data from ring buffer.");
				ring_buf_reset(&uart_rx_rb);
				return;
			}
			parse_error(buf, 8);
		} else {
			if (!ring_buf_is_empty(&incomplete_rb)) {
				ring_buf_put(&incomplete_rb, (uint8_t *)&head, 2);
			}
			if (head == incomplete_target_tail) {
				uint8_t *buf;
				int len = ring_buf_get_claim(&incomplete_rb, &buf,
							     ring_buf_size_get(&incomplete_rb));
				if (buf == NULL) {
					LOG_ERR("Failed to get data from incomplete ring buffer.");
					ring_buf_reset(&incomplete_rb);
					incomplete_target_tail = 0;
					return;
				}
				if (head == SYNC_FRAME_TAIL) {
					int16_t pack_len = -1;
					uart_sync_pack_t *pack = NULL;
					uint16_t id = GET_16BITS(buf, 2);
					for (int i = 0; i < sync_pack_cnt; i++) {
						if (sync_pack[i].ID == id) {
							pack_len = sync_pack[i].len + 8;
							pack = &sync_pack[i];
							break;
						}
					}
					if (len != pack_len) {
						LOG_ERR("Incomplete SYNC frame.");
						ring_buf_reset(&incomplete_rb);
						incomplete_target_tail = 0;
						continue;
					}
					parse_sync(buf, pack, len);
					ring_buf_reset(&incomplete_rb);
					incomplete_target_tail = 0;
					ring_buf_get_finish(&incomplete_rb, len);
				} else if (head == FUNC_FRAME_TAIL) {
					parse_func(buf, len);
					ring_buf_reset(&incomplete_rb);
					incomplete_target_tail = 0;
					ring_buf_get_finish(&incomplete_rb, len);
				} else if (head == ERROR_FRAME_TAIL) {
					parse_error(buf, len);
					ring_buf_reset(&incomplete_rb);
					incomplete_target_tail = 0;
					ring_buf_get_finish(&incomplete_rb, len);
				} else {
					LOG_ERR("Invalid frame tail.");
					ring_buf_reset(&incomplete_rb);
					incomplete_target_tail = 0;
				}
			}
			continue;
		}
	}
}

void uart_trans_sync_flush(uart_sync_pack_t *pack)
{
	if (pack == NULL) {
		LOG_ERR("Sync pack is NULL.");
		return;
	}
	memcpy(pack->buf + SYNC_DATA_IDX, pack->data, pack->len);
	GET_16BITS(pack->buf, SYNC_HEAD_IDX) = SYNC_FRAME_HEAD;
	GET_16BITS(pack->buf, SYNC_ID_IDX) = pack->ID;
	pack->crc = crc8_ccitt(0x00, pack->buf + SYNC_ID_IDX, pack->len + 2);
	GET_8BITS(pack->buf, pack->len + SYNC_CRC_OFFSET) = pack->crc;
	GET_8BITS(pack->buf, pack->len + SYNC_CRC_OFFSET + 1) = 0x00;
	GET_16BITS(pack->buf, pack->len + SYNC_TAIL_OFFSET) = SYNC_FRAME_TAIL;

	LOG_HEXDUMP_DBG(pack->buf, pack->len + 8, "Sync data to send:");

	uart_tx(uart_trans_dev, pack->buf, pack->len + 8, 10);
}

uart_sync_pack_t *uart_trans_sync_add(uart_trans_data_t *data, uint16_t ID, size_t len,
				      uart_trans_cb_t cb)
{
	if (sync_pack_cnt >= MAX_PACK_COUNT) {
		LOG_ERR("Sync pack count exceeds maximum.");
		return NULL;
	}
	sync_pack[sync_pack_cnt].data = data;
	sync_pack[sync_pack_cnt].ID = sys_cpu_to_be16(ID);
	sync_pack[sync_pack_cnt].len = len;
	sync_pack[sync_pack_cnt].cb = cb;
	sync_pack[sync_pack_cnt].buf = malloc(len + 8);
	if (sync_pack[sync_pack_cnt].buf == NULL) {
		LOG_ERR("Failed to allocate memory for SYNC frame.");
		return NULL;
	}
	sync_pack_cnt++;
	if (cb) {
		cb(SYNC_PACK_STATUS_READ);
	}
	uart_trans_sync_flush(&sync_pack[sync_pack_cnt - 1]);

	return &sync_pack[sync_pack_cnt - 1];
}

void uart_trans_func_add(uint16_t header, uart_trans_func_t cb)
{
	if (cb == NULL) {
		LOG_ERR("Callback function is NULL.");
		return;
	}
	if (func_cnt >= MAX_PACK_COUNT) {
		LOG_ERR("Func count exceeds maximum.");
		return;
	}
	func_table[func_cnt].id = sys_cpu_to_be16(header);
	func_table[func_cnt].cb = cb;
	func_cnt++;
}

void uart_trans_func_remove(uint16_t header)
{
	for (int i = 0; i < func_cnt; i++) {
		if (func_table[i].id == header) {
			func_cnt--;
			func_table[i] = func_table[func_cnt];
			return;
		}
	}
	LOG_ERR("Cannot find corresponding ID from func table.");
	return;
}

// async serial callback
static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data)
{
	struct device *uart = user_data;
	int err;

	switch (evt->type) {
	case UART_TX_DONE:
		// LOG_INF("Tx sent %d bytes\n", evt->data.tx.len);
		break;

	case UART_TX_ABORTED:
		// LOG_INF("Tx aborted. Is the serial fast enough?\n");
		break;

	case UART_RX_RDY: {
		LOG_DBG("Received data %d bytes\n", evt->data.rx.len);

		if (evt->data.rx.len != 0) {
			last_receive = k_cycle_get_32();
		}

		ring_buf_put(&uart_rx_rb, evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);

		break;
	}

	case UART_RX_BUF_REQUEST: {
		uint8_t *buf;

		err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
		if (err != 0) {
			LOG_ERR("Failed to allocate memory for RX buffer.");
			uart_rx_disable(uart_trans_dev);
			error_handle(0, TOOHIGH_FREQ);
			rx_throttle = true;
			return;
		}

		err = uart_rx_buf_rsp(uart, buf, SYNC_RX_BUF_SIZE);
		if (err != 0) {
			LOG_ERR("Failed to provide new buffer.");
			k_mem_slab_free(&uart_rx_slab, (void **)&buf);
			uart_rx_disable(uart_trans_dev);
			error_handle(0, TOOHIGH_FREQ);
			rx_throttle = true;
			return;
		}
		break;
	}

	case UART_RX_BUF_RELEASED:
		k_mem_slab_free(&uart_rx_slab, (void *)evt->data.rx_buf.buf);
		if (rx_throttle) {
			uint8_t *buf;
			err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
			if (err != 0) {
				LOG_ERR("Failed to allocate memory for RX buffer.");
				uart_rx_disable(uart_trans_dev);
				error_handle(0, TOOHIGH_FREQ);
				return;
			}
			rx_throttle = false;
			err = uart_rx_enable(uart_trans_dev, buf, SYNC_RX_BUF_SIZE,
					     RX_INACTIVE_TIMEOUT);
			if (err != 0) {
				LOG_ERR("Failed to enable RX.");
				k_mem_slab_free(&uart_rx_slab, (void **)&buf);
				uart_rx_disable(uart_trans_dev);
				error_handle(0, TOOHIGH_FREQ);
				return;
			}
			LOG_INF("RX buffer released, re-enabled RX.");
		}
		break;

	case UART_RX_DISABLED:
		break;

	case UART_RX_STOPPED:
		break;
	}
}

#if CONFIG_USB_UART_ASYNC_ADAPTER
UART_ASYNC_ADAPTER_INST_DEFINE(async_adapter);
#else
static const struct device *const async_adapter;
#endif

// 初始化函数
int uart_trans_init(void)
{
	if (!device_is_ready(uart_trans_dev)) {
		LOG_ERR("UART device not ready");
		return -ENODEV;
	}

	int err = 0;
	if (IS_ENABLED(CONFIG_USB_DEVICE_STACK)) {
		int err = usb_enable(NULL);
		if (err && (err != -EALREADY)) {
			LOG_INF("Failed to enable USB\n");
			return -EIO;
		}
	}

	if (IS_ENABLED(CONFIG_USB_UART_ASYNC_ADAPTER)) {
		/* Implement API adapter */
		uart_async_adapter_init(async_adapter, uart_trans_dev);
		uart_trans_dev = (struct device *)async_adapter;
	}

	GET_16BITS(error_frame, ERROR_HEAD_IDX) = ERROR_FRAME_HEAD;
	GET_16BITS(error_frame, ERROR_TAIL_IDX) = ERROR_FRAME_TAIL;
	GET_8BITS(error_frame, ERROR_ID_IDX) = HEARTBEAT_ID;
	GET_8BITS(error_frame, ERROR_CODE_IDX) = 0x00;

	err = uart_callback_set(uart_trans_dev, uart_callback, (void *)uart_trans_dev);

	// allocate buffer and start rx
	uart_trans_data_t *buf;
	err = k_mem_slab_alloc(&uart_rx_slab, (void **)&buf, K_NO_WAIT);
	__ASSERT(err == 0, "Failed to alloc slab");
	err = uart_rx_enable(uart_trans_dev, buf, BUF_SIZE, RX_INACTIVE_TIMEOUT);
	__ASSERT(err == 0, "Failed to enable rx");

	uart_initialized = true;

	return 0;
}

// 主循环处理函数
void uart_trans_process(void)
{
	int cnt = 0;
	while (1) {
		k_msleep(2);
		cnt++;
		if (uart_initialized && cnt % 50 == 0) {
			error_handle(HEARTBEAT_ID, HEART_BEAT);
			if (online &&
			    (k_cyc_to_ms_near32(abs((int)last_heart_beat - (int)last_receive))) >
				    300) {
				online = false;
				LOG_ERR("Connection lost.");
			} else if (!online &&
				   k_cyc_to_ms_near32(k_cycle_get_32() - last_receive) < 300) {
				online = true;
			}
		}
		process_received_data();
	}
}

K_THREAD_DEFINE(uart_trans_thread, 2048, uart_trans_process, NULL, NULL, NULL, 1, 0, 0);
