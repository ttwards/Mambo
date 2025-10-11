#ifndef ARES_PLOTTER_PROTOCOL_H
#define ARES_PLOTTER_PROTOCOL_H

#include <zephyr/net_buf.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include "ares/protocol/ares_protocol.h"
#include <stddef.h>
#include <stdint.h>

// Protocol API functions
void ares_plotter_protocol_handle(struct AresProtocol *protocol, struct net_buf *buf);
void ares_plotter_protocol_handle_byte(struct AresProtocol *protocol, uint8_t byte);
void ares_plotter_protocol_event(struct AresProtocol *protocol, enum AresProtocolEvent event);
int ares_plotter_protocol_init(struct AresProtocol *protocol);

// --- 用户配置区 User Configuration Section ---

// 定义MCU能支持同时监控的最大变量数量
// Define the maximum number of variables the MCU can monitor simultaneously
#ifndef CONFIG_ARESPLOT_MAX_VARS_TO_MONITOR
#define ARESPLOT_MAX_VARS_TO_MONITOR (10)
#else
#define ARESPLOT_MAX_VARS_TO_MONITOR CONFIG_ARESPLOT_MAX_VARS_TO_MONITOR
#endif

// 定义接收和发送缓冲区的最大大小 (应大于最大可能的帧大小)
// Define max size for receive and transmit buffers (should be larger than max possible frame size)
#ifndef CONFIG_ARESPLOT_SHARED_BUFFER_SIZE
#define ARESPLOT_SHARED_BUFFER_SIZE (128)
#else
#define ARESPLOT_SHARED_BUFFER_SIZE CONFIG_ARESPLOT_SHARED_BUFFER_SIZE
#endif

// 是否启用可选的 CMD_ERROR_REPORT 功能 (1: 启用, 0: 禁用)
// Enable optional CMD_ERROR_REPORT feature (1: enable, 0: disable)
#ifdef CONFIG_ARESPLOT_ENABLE_ERROR_REPORT
#define ARESPLOT_ENABLE_ERROR_REPORT (1)
#else
#define ARESPLOT_ENABLE_ERROR_REPORT (0)
#endif

// 默认采样周期 (毫秒) - 如果 CMD_SET_SAMPLE_RATE 未被调用
// Default sampling period (milliseconds) - if CMD_SET_SAMPLE_RATE is not called
#ifndef CONFIG_ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS
#define ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS (10)
#else
#define ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS CONFIG_ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS
#endif

// --- 协议常量 Protocol Constants ---
#define ARESPLOT_SOP (0xA5) // 帧起始符 Start of Packet
#define ARESPLOT_EOP (0x5A) // 帧结束符 End of Packet

// PC -> MCU 命令ID
#define ARESPLOT_CMD_START_MONITOR   (0x01) // 请求开始/更新/停止监控变量
#define ARESPLOT_CMD_SET_VARIABLE    (0x02) // 请求设置变量值
#define ARESPLOT_CMD_SET_SAMPLE_RATE (0x03) // (可选) 设置采样率

// MCU -> PC 命令ID
#define ARESPLOT_CMD_MONITOR_DATA (0x81) // 发送监控数据
#define ARESPLOT_CMD_ACK          (0x82) // 命令确认/应答
#if ARESPLOT_ENABLE_ERROR_REPORT
#define ARESPLOT_CMD_ERROR_REPORT (0x8F) // (可选) MCU主动错误报告
#endif

// AresOriginalType_t 枚举 (变量原始数据类型)
typedef enum {
	ARES_TYPE_INT8 = 0x00,
	ARES_TYPE_UINT8 = 0x01,
	ARES_TYPE_INT16 = 0x02,
	ARES_TYPE_UINT16 = 0x03,
	ARES_TYPE_INT32 = 0x04,
	ARES_TYPE_UINT32 = 0x05,
	ARES_TYPE_FLOAT32 = 0x06,
	ARES_TYPE_FLOAT64 = 0x07, // 注意: MCU端通常转换为float32发送
	ARES_TYPE_BOOL = 0x08
} aresplot_original_type_t;

// ACK状态码 Status codes for CMD_ACK
typedef enum {
	ARES_STATUS_OK = 0x00,                      // 成功
	ARES_STATUS_ERROR_CHECKSUM = 0x01,          // 接收到的帧校验和错误
	ARES_STATUS_ERROR_UNKNOWN_CMD = 0x02,       // 未知命令
	ARES_STATUS_ERROR_INVALID_PAYLOAD = 0x03,   // Payload 格式或长度错误
	ARES_STATUS_ERROR_ADDR_INVALID = 0x04,      // 无效地址 (如对齐、范围)
	ARES_STATUS_ERROR_TYPE_UNSUPPORTED = 0x05,  // 不支持的数据类型
	ARES_STATUS_ERROR_RATE_UNACHIEVABLE = 0x06, // 无法达到请求的采样率
	ARES_STATUS_ERROR_MCU_BUSY_OR_LIMIT = 0x07, // MCU 繁忙或内部资源限制
	ARES_STATUS_ERROR_GENERAL_FAIL = 0xFF       // 通用失败
} aresplot_ack_status_t;

// Parser states
enum aresplot_parser_state {
	ARES_RX_STATE_WAIT_SOP,
	ARES_RX_STATE_WAIT_CMD,
	ARES_RX_STATE_WAIT_LEN1,
	ARES_RX_STATE_WAIT_LEN2,
	ARES_RX_STATE_WAIT_PAYLOAD,
	ARES_RX_STATE_WAIT_CHECKSUM,
	ARES_RX_STATE_WAIT_EOP
};

// 存储单个监控变量信息的结构体
typedef struct {
	void *ptr; // 指向变量内存地址的指针 Pointer to the variable's memory address
	aresplot_original_type_t type; // 变量的原始数据类型 Original data type of the variable
	const char *name;              // Optional variable name
} aresplot_var_info_t;

// Plotter protocol data structure
struct aresplot_protocol_data {
	const char *name;

	uint8_t tx_buf_len;

	// Parser state
	enum aresplot_parser_state state;
	uint8_t rx_buffer[ARESPLOT_SHARED_BUFFER_SIZE];
	uint16_t rx_payload_len;
	uint16_t rx_payload_idx;
	uint8_t rx_cmd;
	uint8_t rx_checksum_calculated;

	// Variable monitoring
	aresplot_var_info_t monitor_vars[ARESPLOT_MAX_VARS_TO_MONITOR];
	uint8_t num_monitor_vars;
	bool monitoring_active;

	// Timing
	uint32_t sample_period_ms;
	uint32_t last_sample_time;

	// Transmit buffer
	uint8_t tx_buffer[ARESPLOT_SHARED_BUFFER_SIZE];

	// Connection state
	bool online;
	uint32_t last_receive_time;

	struct k_timer sample_timer;
	struct k_mutex vars_mutex;

#if ARESPLOT_ENABLE_ERROR_REPORT
	// 错误报告发送相关
	volatile uint8_t error_report_pending;                          // 是否有错误报告等待发送
	uint8_t error_report_code_to_send;                              // 要发送的错误码
	char error_report_msg_to_send[ARESPLOT_SHARED_BUFFER_SIZE - 7]; // 错误消息缓冲区
	uint8_t error_report_msg_len_to_send;                           // 错误消息长度
#endif
};

// User API functions
int plotter_add_variable(struct AresProtocol *protocol, void *ptr, aresplot_original_type_t type,
			 const char *name);
int plotter_remove_variable(struct AresProtocol *protocol, void *ptr);
int plotter_set_sample_rate(struct AresProtocol *protocol, uint32_t period_ms);

#if ARESPLOT_ENABLE_ERROR_REPORT
/**
 * @brief (可选) 发送一个错误报告给上位机
 * (Optional) Sends an error report to the host PC.
 */
int aresplot_report_error(struct AresProtocol *protocol, uint8_t error_code, const char *message,
			  uint8_t msg_len);
#endif

// Protocol definition macro
#define PLOTTER_PROTOCOL_DEFINE(Protocol_name)                                                     \
	struct AresProtocolAPI Protocol_name##_api = {                                             \
		.handle = ares_plotter_protocol_handle,                                            \
		.handle_byte = ares_plotter_protocol_handle_byte,                                  \
		.event = ares_plotter_protocol_event,                                              \
		.init = ares_plotter_protocol_init,                                                \
	};                                                                                         \
	struct aresplot_protocol_data Protocol_name##_data = {                                     \
		.name = #Protocol_name,                                                            \
		.state = ARES_RX_STATE_WAIT_SOP,                                                   \
		.rx_payload_len = 0,                                                               \
		.rx_payload_idx = 0,                                                               \
		.rx_cmd = 0,                                                                       \
		.rx_checksum_calculated = 0,                                                       \
		.num_monitor_vars = 0,                                                             \
		.monitoring_active = true,                                                         \
		.sample_period_ms = 1000 / CONFIG_ARESPLOT_FREQ,                                   \
		.last_sample_time = 0,                                                             \
		.online = false,                                                                   \
		.last_receive_time = 0,                                                            \
	};                                                                                         \
	struct AresProtocol Protocol_name = {                                                      \
		.name = #Protocol_name,                                                            \
		.api = &Protocol_name##_api,                                                       \
		.priv_data = &Protocol_name##_data,                                                \
	};

#endif // ARES_PLOTTER_PROTOCOL_H
