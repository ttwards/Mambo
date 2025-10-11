#ifndef INTERBOARD_H_
#define INTERBOARD_H_

#include <zephyr/types.h>
#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif
#define datalen             16
#define RECEIVE_ACK         0x0A
#define SPI_BUF_SIZE        24
#define MSG_QUEUE_SIZE      10
#define SPI_SEND_STACK_SIZE 2048
#define SPI_SEND_PRIORITY   -1
typedef struct {
	uint8_t sync;     // 同步头0x5A（固定值）
	uint8_t ctrl;     // [7]主1从0 [6]是否为重发
	uint8_t msg_type; // 消息类型
	uint8_t ack;      // 应答
} Frame_Header;
// Frame structure
typedef struct {
	Frame_Header frame_header; // Start byte to indicate the beginning of a frame
	uint8_t datas[datalen];    // 数据区

	uint16_t crc16;    // CRC16校验（覆盖帧头到datas）
	uint16_t end_mark; // 帧尾标识0x0D0A（固定值
} Interboard_Frame;

typedef enum {
	INTERBOARD_MSG_CAN,
	INTERBOARD_MSG_IMU,

} InterboardMsgType;
// 为消息队列设计的数据结构
typedef struct {
	uint8_t msg_type;
	uint8_t data[16];
} Frame_Data;
// 为信号量和回调函数和线程传递数据的数据结构
typedef struct {
	struct k_sem data_ready;
	uint8_t *rxdata;
} Interboard_Context;

uint16_t calculate_crc16(const uint8_t *data, size_t length);
void build_txbuf(uint8_t *tx_buffer, InterboardMsgType msg_type, uint8_t *data, bool is_retransmit);
void process_rxbuf(uint8_t *rx_buffer, InterboardMsgType msg_type, uint8_t *data,
		   bool is_retransmit);

void interboard_rx_data_handler(struct k_work *work);

void interboardrx_callback(const struct device *spi_dev, int error_code, void *data);

void interboard_tx_data_handler(struct k_work *work);
void interboard_tx_isr_handler(struct k_timer *dummy);

void interboard_init(struct device *spi_dev);
int interboard_transceive(Frame_Data *tx_data, Frame_Data *rx_data);

#ifdef __cplusplus
}
#endif

#endif // INTERBOARD_H_
