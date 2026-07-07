#ifndef ARES_MQTTLITE_PROTOCOL_H
#define ARES_MQTTLITE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <zephyr/kernel.h>

#include <ares/protocol/ares_protocol.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ares_mqttlite_qos {
	ARES_MQTTLITE_QOS0 = 0,
	ARES_MQTTLITE_QOS1 = 1,
	ARES_MQTTLITE_QOS2 = 2,
};

enum ares_mqttlite_publish_status {
	ARES_MQTTLITE_PUBLISH_ACKED = 0,
	ARES_MQTTLITE_PUBLISH_TIMEOUT = -ETIMEDOUT,
};

typedef void (*ares_mqttlite_msg_cb_t)(struct AresProtocol *protocol, const char *topic,
				       const uint8_t *payload, uint16_t payload_len,
				       enum ares_mqttlite_qos qos, void *user_data);

typedef void (*ares_mqttlite_publish_cb_t)(struct AresProtocol *protocol, uint16_t packet_id,
					   int status, void *user_data);

struct ares_mqttlite_publish_buffer {
	struct net_buf *buf;
	uint8_t *payload;
	uint16_t payload_len;
};

int ares_mqttlite_register_topic(struct AresProtocol *protocol, uint16_t topic_id,
				 const char *topic);

int ares_mqttlite_subscribe(struct AresProtocol *protocol, const char *topic_filter,
			    ares_mqttlite_msg_cb_t cb, void *user_data);

int ares_mqttlite_subscribe_id(struct AresProtocol *protocol, uint16_t topic_id,
			       ares_mqttlite_msg_cb_t cb, void *user_data);

int ares_mqttlite_unsubscribe(struct AresProtocol *protocol, const char *topic_filter);

int ares_mqttlite_unsubscribe_id(struct AresProtocol *protocol, uint16_t topic_id);

int ares_mqttlite_publish(struct AresProtocol *protocol, const char *topic, const uint8_t *payload,
			  uint16_t payload_len, enum ares_mqttlite_qos qos,
			  ares_mqttlite_publish_cb_t cb, void *user_data);

int ares_mqttlite_publish_id(struct AresProtocol *protocol, uint16_t topic_id,
			     const uint8_t *payload, uint16_t payload_len,
			     enum ares_mqttlite_qos qos, ares_mqttlite_publish_cb_t cb,
			     void *user_data);

int ares_mqttlite_publish_prepare_id(struct AresProtocol *protocol, uint16_t topic_id,
				     uint16_t payload_len,
				     struct ares_mqttlite_publish_buffer *pub);

int ares_mqttlite_publish_commit(struct AresProtocol *protocol,
				 struct ares_mqttlite_publish_buffer *pub);

void ares_mqttlite_publish_abort(struct ares_mqttlite_publish_buffer *pub);

int ares_mqttlite_ping(struct AresProtocol *protocol);

void ares_mqttlite_protocol_handle(struct AresProtocol *protocol, struct net_buf *buf);
void ares_mqttlite_protocol_handle_byte(struct AresProtocol *protocol, uint8_t byte);
void ares_mqttlite_protocol_event(struct AresProtocol *protocol, enum AresProtocolEvent event);
int ares_mqttlite_protocol_init(struct AresProtocol *protocol);

struct ares_mqttlite_subscription {
	bool use_topic_id;
	uint16_t topic_id;
	char topic_filter[CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN + 1];
	ares_mqttlite_msg_cb_t cb;
	void *user_data;
};

struct ares_mqttlite_topic_entry {
	bool used;
	uint16_t topic_id;
	char topic[CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN + 1];
};

enum ares_mqttlite_tx_state {
	ARES_MQTTLITE_TX_UNUSED,
	ARES_MQTTLITE_TX_WAIT_PUBACK,
	ARES_MQTTLITE_TX_WAIT_PUBREC,
	ARES_MQTTLITE_TX_WAIT_PUBCOMP,
};

struct ares_mqttlite_tx_inflight {
	enum ares_mqttlite_tx_state state;
	uint16_t packet_id;
	enum ares_mqttlite_qos qos;
	uint8_t retries;
	int64_t last_send_ms;
	ares_mqttlite_publish_cb_t cb;
	void *user_data;
	bool use_topic_id;
	uint16_t topic_id;
	uint16_t topic_len;
	uint16_t payload_len;
	char topic[CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN + 1];
	uint8_t payload[CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE];
};

struct ares_mqttlite_rx_qos2 {
	bool used;
	uint16_t packet_id;
	bool use_topic_id;
	uint16_t topic_id;
	uint16_t topic_len;
	uint16_t payload_len;
	char topic[CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN + 1];
	uint8_t payload[CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE];
};

struct ares_mqttlite_protocol_data {
	const char *name;
	bool online;
	uint16_t next_packet_id;
	struct k_mutex lock;
	struct k_work_delayable retry_work;
	struct AresProtocol *protocol;
	struct ares_mqttlite_topic_entry topics[CONFIG_ARES_MQTTLITE_MAX_TOPICS];
	struct ares_mqttlite_subscription subscriptions[CONFIG_ARES_MQTTLITE_MAX_SUBSCRIPTIONS];
	struct ares_mqttlite_tx_inflight tx[CONFIG_ARES_MQTTLITE_MAX_INFLIGHT];
	struct ares_mqttlite_rx_qos2 rx_qos2[CONFIG_ARES_MQTTLITE_MAX_INFLIGHT];
	uint8_t rx_buf[CONFIG_ARES_MQTTLITE_MAX_FRAME_SIZE];
	uint16_t rx_pos;
	uint16_t rx_expected;
};

#define ARES_MQTTLITE_PROTOCOL_DEFINE(Protocol_name)                                               \
	struct AresProtocolAPI Protocol_name##_api = {                                             \
		.handle = ares_mqttlite_protocol_handle,                                           \
		.handle_byte = ares_mqttlite_protocol_handle_byte,                                 \
		.event = ares_mqttlite_protocol_event,                                             \
		.init = ares_mqttlite_protocol_init,                                               \
	};                                                                                         \
	struct ares_mqttlite_protocol_data Protocol_name##_data = {                                \
		.name = #Protocol_name,                                                            \
	};                                                                                         \
	struct AresProtocol Protocol_name = {                                                      \
		.name = #Protocol_name,                                                            \
		.api = &Protocol_name##_api,                                                       \
		.priv_data = &Protocol_name##_data,                                                \
	}

#ifdef __cplusplus
}
#endif

#endif
