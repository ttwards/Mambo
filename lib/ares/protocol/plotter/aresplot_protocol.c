// aresplot_mcu.c

#include "ares/protocol/plotter/aresplot_protocol.h"
#include "ares/interface/ares_interface.h"
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <string.h>
#include <sys/errno.h>

LOG_MODULE_REGISTER(aresplot, LOG_LEVEL_INF);

// --- 内部辅助函数 Internal Helper Functions ---

/**
 * @brief 计算帧校验和 (从CMD到PAYLOAD尾)
 * Calculates the frame checksum (from CMD to end of PAYLOAD).
 */
static uint8_t calculate_checksum(uint8_t cmd, uint16_t len, const uint8_t *payload)
{
	uint8_t checksum = 0;
	checksum ^= cmd;
	checksum ^= (uint8_t)(len & 0xFF);
	checksum ^= (uint8_t)((len >> 8) & 0xFF);
	for (uint16_t i = 0; i < len; ++i) {
		checksum ^= payload[i];
	}
	return checksum;
}

/**
 * @brief 组装并发送一个完整的帧
 * Assembles and sends a complete frame.
 */
static int assemble_and_send_frame_internal(struct AresProtocol *protocol, uint8_t cmd,
					    const uint8_t *payload, uint16_t len)
{
	struct aresplot_protocol_data *data = protocol->priv_data;
	uint16_t frame_idx = 0;
	uint16_t total_frame_len = 1 + 1 + 2 + len + 1 + 1; // SOP+CMD+LEN+Payload+CS+EOP

	if (total_frame_len > ARESPLOT_SHARED_BUFFER_SIZE) {
		LOG_ERR("%s Frame too large to assemble: %d > %d", data->name, total_frame_len,
			ARESPLOT_SHARED_BUFFER_SIZE);
		return -ENOMEM;
	}

	// 1. SOP
	data->tx_buffer[frame_idx++] = ARESPLOT_SOP;
	// 2. CMD
	data->tx_buffer[frame_idx++] = cmd;
	// 3. LEN (Little Endian)
	data->tx_buffer[frame_idx++] = (uint8_t)(len & 0xFF);
	data->tx_buffer[frame_idx++] = (uint8_t)((len >> 8) & 0xFF);
	// 4. PAYLOAD
	if (payload && len > 0) {
		memcpy(&data->tx_buffer[frame_idx], payload, len);
		frame_idx += len;
	}
	// 5. CHECKSUM
	data->tx_buffer[frame_idx++] = calculate_checksum(cmd, len, payload);
	// 6. EOP
	data->tx_buffer[frame_idx++] = ARESPLOT_EOP;

	LOG_DBG("%s Sending frame: cmd=0x%02x, len=%d", data->name, cmd, frame_idx);

	return protocol->interface->api->send_raw(protocol->interface, data->tx_buffer, frame_idx);
}

/**
 * @brief 发送ACK响应
 * Sends an ACK response immediately.
 */
static void send_ack_response(struct AresProtocol *protocol, uint8_t ack_cmd_id,
			      aresplot_ack_status_t status)
{
	uint8_t ack_payload[2];
	ack_payload[0] = ack_cmd_id;
	ack_payload[1] = (uint8_t)status;
	assemble_and_send_frame_internal(protocol, ARESPLOT_CMD_ACK, ack_payload, 2);
}

/**
 * @brief 获取系统时间戳（毫秒）
 */
static uint32_t get_timestamp_ms(void)
{
	return k_uptime_get_32();
}

/**
 * @brief 处理接收到的 CMD_START_MONITOR 命令
 */
static void handle_cmd_start_monitor(struct AresProtocol *protocol)
{
	struct aresplot_protocol_data *data = protocol->priv_data;
	uint8_t num_vars_requested = data->rx_buffer[0];
	aresplot_ack_status_t status = ARES_STATUS_OK;

	LOG_DBG("%s Start monitor request: %d variables", data->name, num_vars_requested);

	k_mutex_lock(&data->vars_mutex, K_FOREVER);
	data->monitoring_active = false;
	data->num_monitor_vars = 0;

	if (num_vars_requested == 0) {
		status = ARES_STATUS_OK;
		LOG_INF("%s Monitor stopped", data->name);
		// 停止定时器
		k_timer_stop(&data->sample_timer);
	} else if (num_vars_requested > ARESPLOT_MAX_VARS_TO_MONITOR) {
		status = ARES_STATUS_ERROR_MCU_BUSY_OR_LIMIT;
		LOG_ERR("%s Too many variables requested: %d > %d", data->name, num_vars_requested,
			ARESPLOT_MAX_VARS_TO_MONITOR);
	} else {
		if (data->rx_payload_len != (1 + num_vars_requested * 5)) {
			status = ARES_STATUS_ERROR_INVALID_PAYLOAD;
			LOG_ERR("%s Invalid payload length: %d != %d", data->name,
				data->rx_payload_len, (1 + num_vars_requested * 5));
		} else {
			data->num_monitor_vars = num_vars_requested;
			for (uint8_t i = 0; i < data->num_monitor_vars; ++i) {
				uint8_t *p_var_info_payload = &data->rx_buffer[1 + i * 5];

				uint32_t temp_addr = (uint32_t)p_var_info_payload[0] |
						     ((uint32_t)p_var_info_payload[1] << 8) |
						     ((uint32_t)p_var_info_payload[2] << 16) |
						     ((uint32_t)p_var_info_payload[3] << 24);
				data->monitor_vars[i].ptr = (void *)temp_addr;
				data->monitor_vars[i].type =
					(aresplot_original_type_t)p_var_info_payload[4];
				data->monitor_vars[i].name = NULL; // No name in protocol

				LOG_DBG("%s Added variable %d: addr=0x%08x, type=%d", data->name, i,
					temp_addr, p_var_info_payload[4]);
			}
			data->monitoring_active = true;
			data->last_sample_time = get_timestamp_ms();
			status = ARES_STATUS_OK;
			LOG_INF("%s Monitor started with %d variables", data->name,
				num_vars_requested);

			// 启动定时器开始监控
			k_timer_start(&data->sample_timer, K_MSEC(data->sample_period_ms),
				      K_MSEC(data->sample_period_ms));
		}
	}
	k_mutex_unlock(&data->vars_mutex);
	send_ack_response(protocol, ARESPLOT_CMD_START_MONITOR, status);
}

/**
 * @brief 处理接收到的 CMD_SET_VARIABLE 命令
 */
static void handle_cmd_set_variable(struct AresProtocol *protocol)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	if (data->rx_payload_len != 9) {
		LOG_ERR("%s Invalid SET_VARIABLE payload length: %d != 9", data->name,
			data->rx_payload_len);
		send_ack_response(protocol, ARESPLOT_CMD_SET_VARIABLE,
				  ARES_STATUS_ERROR_INVALID_PAYLOAD);
		return;
	}

	void *addr;
	aresplot_original_type_t original_type;
	float float_val;
	uint8_t *p_payload = data->rx_buffer;

	uint32_t temp_addr = (uint32_t)p_payload[0] | ((uint32_t)p_payload[1] << 8) |
			     ((uint32_t)p_payload[2] << 16) | ((uint32_t)p_payload[3] << 24);
	addr = (void *)temp_addr;
	original_type = (aresplot_original_type_t)p_payload[4];

	uint8_t temp_float_bytes[4];
	temp_float_bytes[0] = p_payload[5];
	temp_float_bytes[1] = p_payload[6];
	temp_float_bytes[2] = p_payload[7];
	temp_float_bytes[3] = p_payload[8];
	memcpy(&float_val, temp_float_bytes, sizeof(float));

	LOG_DBG("%s Setting variable: addr=0x%08x, type=%d, value=%f", data->name, temp_addr,
		original_type, (double)float_val);

	k_mutex_lock(&data->vars_mutex, K_FOREVER);
	switch (original_type) {
	case ARES_TYPE_INT8:
		*(volatile int8_t *)addr = (int8_t)float_val;
		break;
	case ARES_TYPE_UINT8:
		*(volatile uint8_t *)addr = (uint8_t)float_val;
		break;
	case ARES_TYPE_INT16:
		*(volatile int16_t *)addr = (int16_t)float_val;
		break;
	case ARES_TYPE_UINT16:
		*(volatile uint16_t *)addr = (uint16_t)float_val;
		break;
	case ARES_TYPE_INT32:
		*(volatile int32_t *)addr = (int32_t)float_val;
		break;
	case ARES_TYPE_UINT32:
		*(volatile uint32_t *)addr = (uint32_t)float_val;
		break;
	case ARES_TYPE_FLOAT32:
		*(volatile float *)addr = float_val;
		break;
	case ARES_TYPE_BOOL:
		*(volatile uint8_t *)addr = (float_val != 0.0f) ? 1 : 0;
		break;
	default:
		LOG_ERR("%s Unsupported variable type: %d", data->name, original_type);
		k_mutex_unlock(&data->vars_mutex);
		send_ack_response(protocol, ARESPLOT_CMD_SET_VARIABLE,
				  ARES_STATUS_ERROR_TYPE_UNSUPPORTED);
		return;
	}
	k_mutex_unlock(&data->vars_mutex);
	send_ack_response(protocol, ARESPLOT_CMD_SET_VARIABLE, ARES_STATUS_OK);
}

/**
 * @brief 处理接收到的 CMD_SET_SAMPLE_RATE 命令
 */
static void handle_cmd_set_sample_rate(struct AresProtocol *protocol)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	if (data->rx_payload_len != 4) {
		send_ack_response(protocol, ARESPLOT_CMD_SET_SAMPLE_RATE,
				  ARES_STATUS_ERROR_INVALID_PAYLOAD);
		return;
	}

	uint32_t rate_hz;
	uint8_t *p_payload = data->rx_buffer;

	rate_hz = (uint32_t)p_payload[0] | ((uint32_t)p_payload[1] << 8) |
		  ((uint32_t)p_payload[2] << 16) | ((uint32_t)p_payload[3] << 24);

	k_mutex_lock(&data->vars_mutex, K_FOREVER);
	if (rate_hz == 0) {
		data->sample_period_ms = ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS;
	} else {
		uint32_t period_ms = 1000 / rate_hz;
		if (period_ms == 0 && rate_hz != 0) {
			period_ms = 1;
		}
		data->sample_period_ms = period_ms;
	}
	data->last_sample_time = get_timestamp_ms();
	k_mutex_unlock(&data->vars_mutex);

	send_ack_response(protocol, ARESPLOT_CMD_SET_SAMPLE_RATE, ARES_STATUS_OK);
}

/**
 * @brief 处理一个完整的、校验通过的帧
 */
static void process_received_frame(struct AresProtocol *protocol)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	switch (data->rx_cmd) {
	case ARESPLOT_CMD_START_MONITOR:
		handle_cmd_start_monitor(protocol);
		break;
	case ARESPLOT_CMD_SET_VARIABLE:
		handle_cmd_set_variable(protocol);
		break;
	case ARESPLOT_CMD_SET_SAMPLE_RATE:
		handle_cmd_set_sample_rate(protocol);
		break;
	default:
		send_ack_response(protocol, data->rx_cmd, ARES_STATUS_ERROR_UNKNOWN_CMD);
		break;
	}
}

/**
 * @brief 发送监控数据
 */
static void send_monitor_data(struct AresProtocol *protocol)
{
	struct aresplot_protocol_data *data = protocol->priv_data;
	uint32_t timestamp;
	uint16_t payload_idx = 0;
	float temp_float_val;
	uint8_t current_num_vars;
	aresplot_var_info_t local_monitor_vars[ARESPLOT_MAX_VARS_TO_MONITOR];
	uint8_t monitor_payload[4 + ARESPLOT_MAX_VARS_TO_MONITOR * 4];

	k_mutex_lock(&data->vars_mutex, K_FOREVER);
	if (!data->monitoring_active || data->num_monitor_vars == 0) {
		k_mutex_unlock(&data->vars_mutex);
		return;
	}
	current_num_vars = data->num_monitor_vars;
	for (uint8_t i = 0; i < current_num_vars; ++i) {
		local_monitor_vars[i] = data->monitor_vars[i];
	}
	k_mutex_unlock(&data->vars_mutex);

	timestamp = get_timestamp_ms();

	monitor_payload[payload_idx++] = (uint8_t)(timestamp & 0xFF);
	monitor_payload[payload_idx++] = (uint8_t)((timestamp >> 8) & 0xFF);
	monitor_payload[payload_idx++] = (uint8_t)((timestamp >> 16) & 0xFF);
	monitor_payload[payload_idx++] = (uint8_t)((timestamp >> 24) & 0xFF);

	for (uint8_t i = 0; i < current_num_vars; ++i) {
		if (local_monitor_vars[i].ptr == NULL) {
			temp_float_val = 0.0f;
		} else {
			switch (local_monitor_vars[i].type) {
			case ARES_TYPE_INT8:
				temp_float_val =
					(float)(*(volatile int8_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_UINT8:
				temp_float_val =
					(float)(*(volatile uint8_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_INT16:
				temp_float_val =
					(float)(*(volatile int16_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_UINT16:
				temp_float_val =
					(float)(*(volatile uint16_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_INT32:
				temp_float_val =
					(float)(*(volatile int32_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_UINT32:
				temp_float_val =
					(float)(*(volatile uint32_t *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_FLOAT32:
				temp_float_val = (*(volatile float *)local_monitor_vars[i].ptr);
				break;
			case ARES_TYPE_BOOL:
				temp_float_val = (*(volatile uint8_t *)local_monitor_vars[i].ptr)
							 ? 1.0f
							 : 0.0f;
				break;
			default:
				temp_float_val = 0.0f;
				break;
			}
		}

		uint8_t temp_float_bytes[4];
		memcpy(temp_float_bytes, &temp_float_val, sizeof(float));
		monitor_payload[payload_idx++] = temp_float_bytes[0];
		monitor_payload[payload_idx++] = temp_float_bytes[1];
		monitor_payload[payload_idx++] = temp_float_bytes[2];
		monitor_payload[payload_idx++] = temp_float_bytes[3];
	}

	assemble_and_send_frame_internal(protocol, ARESPLOT_CMD_MONITOR_DATA, monitor_payload,
					 4 + current_num_vars * 4);
}

/**
 * @brief 定时器回调函数用于发送监控数据
 */
static void sample_timer_handler(struct k_timer *timer)
{
	struct AresProtocol *protocol = k_timer_user_data_get(timer);
	send_monitor_data(protocol);
}

// --- 协议 API 函数实现 Protocol API Function Implementations ---

void ares_plotter_protocol_handle_byte(struct AresProtocol *protocol, uint8_t byte)
{
	struct aresplot_protocol_data *data = protocol->priv_data;
	data->last_receive_time = get_timestamp_ms();

	switch (data->state) {
	case ARES_RX_STATE_WAIT_SOP:
		if (byte == ARESPLOT_SOP) {
			data->state = ARES_RX_STATE_WAIT_CMD;
			data->rx_checksum_calculated = 0;
			LOG_DBG("%s Received SOP", data->name);
		}
		break;

	case ARES_RX_STATE_WAIT_CMD:
		data->rx_cmd = byte;
		data->rx_checksum_calculated ^= byte;
		data->state = ARES_RX_STATE_WAIT_LEN1;
		LOG_DBG("%s Received CMD: 0x%02x", data->name, byte);
		break;

	case ARES_RX_STATE_WAIT_LEN1:
		data->rx_payload_len = byte; // LSB
		data->rx_checksum_calculated ^= byte;
		data->state = ARES_RX_STATE_WAIT_LEN2;
		break;

	case ARES_RX_STATE_WAIT_LEN2:
		data->rx_payload_len |= ((uint16_t)byte << 8); // MSB
		data->rx_checksum_calculated ^= byte;
		if (data->rx_payload_len > sizeof(data->rx_buffer)) {
			LOG_ERR("%s Payload too large: %d > %d", data->name, data->rx_payload_len,
				sizeof(data->rx_buffer));
			data->state = ARES_RX_STATE_WAIT_SOP;
		} else if (data->rx_payload_len == 0) {
			data->state = ARES_RX_STATE_WAIT_CHECKSUM;
		} else {
			data->rx_payload_idx = 0;
			data->state = ARES_RX_STATE_WAIT_PAYLOAD;
		}
		LOG_DBG("%s Received LEN: %d", data->name, data->rx_payload_len);
		break;

	case ARES_RX_STATE_WAIT_PAYLOAD:
		data->rx_buffer[data->rx_payload_idx++] = byte;
		data->rx_checksum_calculated ^= byte;
		if (data->rx_payload_idx >= data->rx_payload_len) {
			data->state = ARES_RX_STATE_WAIT_CHECKSUM;
		}
		break;

	case ARES_RX_STATE_WAIT_CHECKSUM:
		if (byte == data->rx_checksum_calculated) {
			data->state = ARES_RX_STATE_WAIT_EOP;
			LOG_DBG("%s Checksum OK", data->name);
		} else {
			LOG_ERR("%s Checksum error: received=0x%02x, calculated=0x%02x", data->name,
				byte, data->rx_checksum_calculated);
			send_ack_response(protocol, data->rx_cmd, ARES_STATUS_ERROR_CHECKSUM);
			data->state = ARES_RX_STATE_WAIT_SOP;
		}
		break;

	case ARES_RX_STATE_WAIT_EOP:
		if (byte == ARESPLOT_EOP) {
			LOG_DBG("%s Received complete frame", data->name);
			process_received_frame(protocol);
		} else {
			LOG_ERR("%s Invalid EOP: 0x%02x", data->name, byte);
		}
		data->state = ARES_RX_STATE_WAIT_SOP;
		break;

	default:
		LOG_ERR("%s Invalid RX state: %d", data->name, data->state);
		data->state = ARES_RX_STATE_WAIT_SOP;
		break;
	}
}

void ares_plotter_protocol_handle(struct AresProtocol *protocol, struct net_buf *buf)
{
	for (uint16_t i = 0; i < buf->len; ++i) {
		ares_plotter_protocol_handle_byte(protocol, buf->data[i]);
	}
}

void ares_plotter_protocol_event(struct AresProtocol *protocol, enum AresProtocolEvent event)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	if (event == ARES_PROTOCOL_EVENT_CONNECTED) {
		LOG_INF("%s Connection established", data->name);
		data->online = true;
	} else if (event == ARES_PROTOCOL_EVENT_DISCONNECTED) {
		LOG_INF("%s Connection lost", data->name);
		data->online = false;
		k_mutex_lock(&data->vars_mutex, K_FOREVER);
		data->monitoring_active = false;
		k_mutex_unlock(&data->vars_mutex);
	}
}

int ares_plotter_protocol_init(struct AresProtocol *protocol)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	LOG_DBG("%s aresplot_protocol_init", data->name);

	data->state = ARES_RX_STATE_WAIT_SOP;
	data->num_monitor_vars = 0;
	data->monitoring_active = false; // 修复：初始化时不启动监控
	data->last_sample_time = 0;
	data->online = false;

	k_mutex_init(&data->vars_mutex);
	k_timer_init(&data->sample_timer, sample_timer_handler, NULL);
	k_timer_user_data_set(&data->sample_timer, protocol);
	// 修复：不在初始化时启动定时器，等待START_MONITOR命令
	k_timer_start(&data->sample_timer, K_MSEC(data->sample_period_ms),
		      K_MSEC(data->sample_period_ms));

#if ARESPLOT_ENABLE_ERROR_REPORT
	data->error_report_pending = 0;
#endif

	LOG_INF("%s AresPlot protocol initialized, waiting for START_MONITOR command", data->name);
	return 0;
}

// --- 用户 API 函数实现 User API Function Implementations ---

int plotter_add_variable(struct AresProtocol *protocol, void *ptr, aresplot_original_type_t type,
			 const char *name)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	k_mutex_lock(&data->vars_mutex, K_FOREVER);

	if (data->num_monitor_vars >= ARESPLOT_MAX_VARS_TO_MONITOR) {
		k_mutex_unlock(&data->vars_mutex);
		return -ENOMEM;
	}

	data->monitor_vars[data->num_monitor_vars].ptr = ptr;
	data->monitor_vars[data->num_monitor_vars].type = type;
	data->monitor_vars[data->num_monitor_vars].name = name;
	data->num_monitor_vars++;

	k_mutex_unlock(&data->vars_mutex);
	return 0;
}

int plotter_remove_variable(struct AresProtocol *protocol, void *ptr)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	k_mutex_lock(&data->vars_mutex, K_FOREVER);

	for (uint8_t i = 0; i < data->num_monitor_vars; i++) {
		if (data->monitor_vars[i].ptr == ptr) {
			// Move the last element to this position
			data->num_monitor_vars--;
			if (i < data->num_monitor_vars) {
				data->monitor_vars[i] = data->monitor_vars[data->num_monitor_vars];
			}
			k_mutex_unlock(&data->vars_mutex);
			return 0;
		}
	}

	k_mutex_unlock(&data->vars_mutex);
	return -ENOENT;
}

int plotter_set_sample_rate(struct AresProtocol *protocol, uint32_t period_ms)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	k_mutex_lock(&data->vars_mutex, K_FOREVER);
	data->sample_period_ms = period_ms;
	k_mutex_unlock(&data->vars_mutex);

	k_timer_stop(&data->sample_timer);
	k_timer_start(&data->sample_timer, K_MSEC(period_ms), K_MSEC(period_ms));

	return 0;
}

#if ARESPLOT_ENABLE_ERROR_REPORT
int aresplot_report_error(struct AresProtocol *protocol, uint8_t error_code, const char *message,
			  uint8_t msg_len)
{
	struct aresplot_protocol_data *data = protocol->priv_data;

	k_mutex_lock(&data->vars_mutex, K_FOREVER);
	if (data->error_report_pending) {
		k_mutex_unlock(&data->vars_mutex);
		return -EBUSY;
	}

	// 确保消息不会导致payload溢出
	if (msg_len >= sizeof(data->error_report_msg_to_send)) {
		msg_len = sizeof(data->error_report_msg_to_send) - 1;
	}

	data->error_report_code_to_send = error_code;
	if (message && msg_len > 0) {
		memcpy(data->error_report_msg_to_send, message, msg_len);
		data->error_report_msg_len_to_send = msg_len;
	} else {
		data->error_report_msg_len_to_send = 0;
	}

	data->error_report_pending = 1;
	k_mutex_unlock(&data->vars_mutex);

	// 立即发送错误报告
	uint8_t error_payload[1 + ARESPLOT_SHARED_BUFFER_SIZE - 7];
	uint16_t error_payload_len = 1 + data->error_report_msg_len_to_send;
	error_payload[0] = error_code;
	if (data->error_report_msg_len_to_send > 0) {
		memcpy(&error_payload[1], data->error_report_msg_to_send,
		       data->error_report_msg_len_to_send);
	}

	int ret = assemble_and_send_frame_internal(protocol, ARESPLOT_CMD_ERROR_REPORT,
						   error_payload, error_payload_len);

	k_mutex_lock(&data->vars_mutex, K_FOREVER);
	data->error_report_pending = 0;
	k_mutex_unlock(&data->vars_mutex);

	return (ret == 0) ? 0 : -EIO;
}

#endif

#if CONFIG_PLOTTER
#include <ares/interface/uart/uart.h>
#include <ares/ares_comm.h>

// UART device definition
#define UART_DEV DT_ALIAS(plot)
static const struct device *_plotter_uart_dev = DEVICE_DT_GET(UART_DEV);

// Protocol and interface definition
PLOTTER_PROTOCOL_DEFINE(_plotter_protocol);
ARES_UART_INTERFACE_DEFINE(_plotter_uart_interface);

void aresplot_auto_init(void)
{
	// Check if UART device is ready
	if (!device_is_ready(_plotter_uart_dev)) {
		LOG_ERR("UART device is not ready");
		return;
	}

	// Initialize UART interface
	ares_uart_init_dev(&_plotter_uart_interface, _plotter_uart_dev);

	// Bind plotter protocol to UART interface
	int ret = ares_bind_interface(&_plotter_uart_interface, &_plotter_protocol);
	if (ret != 0) {
		LOG_ERR("Failed to bind plotter protocol to UART interface: %d", ret);
		return;
	}

	LOG_INF("Plotter protocol is bound to UART interface");
}
SYS_INIT(aresplot_auto_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif
