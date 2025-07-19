// main.c
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/interface/uart/uart.h>
#include <ares/protocol/dual/dual_protocol.h>
#include <ares/ares_comm.h>

#include <zephyr/debug/thread_analyzer.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

#define UART_DEV DT_NODELABEL(usart6)
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEV);

DUAL_PROPOSE_PROTOCOL_DEFINE(dual_protocol);
DUAL_PROPOSE_PROTOCOL_DEFINE(uart_protocol);
ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);
ARES_UART_INTERFACE_DEFINE(uart_interface);

int cnt = 0;
int func_cb(uint32_t param1, uint32_t param2, uint32_t param3)
{
	cnt++;
	LOG_INF("func_cb");
	LOG_INF("params: %2x,%2x,%2x", param1, param2, param3);
	return cnt;
}

void sync_cb(uint32_t status)
{
	LOG_INF("sync_cb: %d", status);
}

void func_ret_cb(uint16_t id, uint16_t req_id, uint32_t ret)
{
	LOG_INF("func_ret_cb: ID %x, req_id %x, ret %x", id, req_id, ret);
}

uint8_t test[59] = {0};
int main(void)
{
	// Initialize the USB stack and our async pipeline.
	// Pass the function that will handle the data.
	// ares_uart_init_dev(&uart_interface, uart_dev);
	int err = ares_bind_interface(&usb_bulk_interface, &dual_protocol);
	// err = ares_bind_interface(&uart_interface, &uart_protocol);
	dual_ret_cb_set(&dual_protocol, (dual_func_ret_cb_t)func_ret_cb);
	dual_func_add(&dual_protocol, 0x1, (dual_trans_func_t)func_cb);
	sync_table_t *pack =
		dual_sync_add(&dual_protocol, 0x1, test, sizeof(test), (dual_trans_cb_t)sync_cb);
	if (err) {
		LOG_ERR("Failed to initialize ARES USB device (%d)", err);
		return 0;
	}

	LOG_INF("ARES Async USB Bulk device is ready.");

	// The main thread is now free to do other things.
	// All USB data processing happens in the background.
	// uint32_t prev_cnt = 0;
	// uint32_t prev_time = k_uptime_get_32();
	int cnt = 0;
	while (1) {
		k_sleep(K_MSEC(100));
		// dual_sync_flush(&dual_protocol, pack);
		dual_func_call(&dual_protocol, 0x1, 0x1, 0x2, cnt);
		// if (prev_time + 1000 < k_uptime_get_32()) {
		// 	// 	// thread_analyzer_print(NULL);
		// 	LOG_INF("cnt: %d, delta: %d", cnt, cnt - prev_cnt);
		// 	prev_time = k_uptime_get_32();
		// }
		// prev_cnt = cnt;

		// You can also call ares_usbd_write() from here to send data proactively.
	}

	return 0;
}