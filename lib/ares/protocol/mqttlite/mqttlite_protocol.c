#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>

#include <ares/interface/ares_interface.h>
#include <ares/protocol/mqttlite/mqttlite_protocol.h>

LOG_MODULE_REGISTER(ares_mqttlite, CONFIG_ARES_MQTTLITE_LOG_LEVEL);

#define MQTL_MAGIC   0x4d51
#define MQTL_VERSION 1

#define MQTL_MAGIC_IDX       0
#define MQTL_VERSION_IDX     2
#define MQTL_TYPE_IDX        3
#define MQTL_FLAGS_IDX       4
#define MQTL_RESERVED_IDX    5
#define MQTL_PACKET_ID_IDX   6
#define MQTL_TOPIC_LEN_IDX   8
#define MQTL_PAYLOAD_LEN_IDX 10
#define MQTL_HEADER_LEN      12

#define MQTL_FLAG_QOS_MASK 0x03
#define MQTL_FLAG_DUP      BIT(2)
#define MQTL_FLAG_TOPIC_ID BIT(3)

#define MQTL_TOPIC_ID_LEN 2

enum mqttlite_frame_type {
	MQTL_FRAME_PUBLISH = 1,
	MQTL_FRAME_PUBACK = 2,
	MQTL_FRAME_PUBREC = 3,
	MQTL_FRAME_PUBREL = 4,
	MQTL_FRAME_PUBCOMP = 5,
	MQTL_FRAME_PING = 6,
	MQTL_FRAME_PONG = 7,
};

static uint16_t frame_topic_len(const uint8_t *frame)
{
	return sys_get_le16(&frame[MQTL_TOPIC_LEN_IDX]);
}

static uint16_t frame_payload_len(const uint8_t *frame)
{
	return sys_get_le16(&frame[MQTL_PAYLOAD_LEN_IDX]);
}

static uint16_t frame_packet_id(const uint8_t *frame)
{
	return sys_get_le16(&frame[MQTL_PACKET_ID_IDX]);
}

static enum ares_mqttlite_qos frame_qos(const uint8_t *frame)
{
	return frame[MQTL_FLAGS_IDX] & MQTL_FLAG_QOS_MASK;
}

static bool frame_uses_topic_id(const uint8_t *frame)
{
	return (frame[MQTL_FLAGS_IDX] & MQTL_FLAG_TOPIC_ID) != 0;
}

static bool valid_qos(enum ares_mqttlite_qos qos)
{
	return qos == ARES_MQTTLITE_QOS0 || qos == ARES_MQTTLITE_QOS1 || qos == ARES_MQTTLITE_QOS2;
}

static const char *find_topic_name_locked(struct ares_mqttlite_protocol_data *data,
					  uint16_t topic_id)
{
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_TOPICS; i++) {
		if (data->topics[i].used && data->topics[i].topic_id == topic_id) {
			return data->topics[i].topic;
		}
	}

	return NULL;
}

static bool topic_matches(const char *filter, const char *topic)
{
	const char *fp = filter;
	const char *tp = topic;

	if (strcmp(filter, "#") == 0) {
		return true;
	}

	while (*fp != '\0' && *tp != '\0') {
		if (*fp == '#') {
			return fp[1] == '\0' && (fp == filter || fp[-1] == '/');
		}
		if (*fp == '+') {
			while (*tp != '\0' && *tp != '/') {
				tp++;
			}
			fp++;
			continue;
		}
		if (*fp != *tp) {
			return false;
		}
		fp++;
		tp++;
	}

	return *fp == *tp || (strcmp(fp, "/#") == 0);
}

static int alloc_packet_id_locked(struct ares_mqttlite_protocol_data *data)
{
	uint16_t packet_id = data->next_packet_id;

	if (packet_id == 0) {
		packet_id = 1;
	}
	data->next_packet_id = packet_id + 1;
	if (data->next_packet_id == 0) {
		data->next_packet_id = 1;
	}

	return packet_id;
}

static struct ares_mqttlite_tx_inflight *find_tx_locked(struct ares_mqttlite_protocol_data *data,
							uint16_t packet_id)
{
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_INFLIGHT; i++) {
		if (data->tx[i].state != ARES_MQTTLITE_TX_UNUSED &&
		    data->tx[i].packet_id == packet_id) {
			return &data->tx[i];
		}
	}

	return NULL;
}

static struct ares_mqttlite_tx_inflight *alloc_tx_locked(struct ares_mqttlite_protocol_data *data)
{
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_INFLIGHT; i++) {
		if (data->tx[i].state == ARES_MQTTLITE_TX_UNUSED) {
			return &data->tx[i];
		}
	}

	return NULL;
}

static struct ares_mqttlite_rx_qos2 *find_rx_qos2_locked(struct ares_mqttlite_protocol_data *data,
							 uint16_t packet_id)
{
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_INFLIGHT; i++) {
		if (data->rx_qos2[i].used && data->rx_qos2[i].packet_id == packet_id) {
			return &data->rx_qos2[i];
		}
	}

	return NULL;
}

static struct ares_mqttlite_rx_qos2 *alloc_rx_qos2_locked(struct ares_mqttlite_protocol_data *data)
{
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_INFLIGHT; i++) {
		if (!data->rx_qos2[i].used) {
			return &data->rx_qos2[i];
		}
	}

	return NULL;
}

static int send_frame(struct AresProtocol *protocol, enum mqttlite_frame_type type,
		      enum ares_mqttlite_qos qos, uint8_t extra_flags, uint16_t packet_id,
		      bool use_topic_id, uint16_t topic_id, const char *topic, uint16_t topic_len,
		      const uint8_t *payload, uint16_t payload_len)
{
	struct AresInterface *interface = protocol->interface;
	struct net_buf *buf;
	uint16_t wire_topic_len = use_topic_id ? MQTL_TOPIC_ID_LEN : topic_len;
	size_t frame_len = MQTL_HEADER_LEN + wire_topic_len + payload_len;

	if (interface == NULL || interface->api == NULL || interface->api->send == NULL ||
	    interface->api->alloc_buf == NULL) {
		return -ENODEV;
	}
	if (frame_len > CONFIG_ARES_MQTTLITE_MAX_FRAME_SIZE) {
		return -EMSGSIZE;
	}

	buf = interface->api->alloc_buf(interface);
	if (buf == NULL) {
		return -ENOMEM;
	}
	if (net_buf_tailroom(buf) < frame_len) {
		net_buf_unref(buf);
		return -EMSGSIZE;
	}

	net_buf_add(buf, frame_len);
	sys_put_le16(MQTL_MAGIC, &buf->data[MQTL_MAGIC_IDX]);
	buf->data[MQTL_VERSION_IDX] = MQTL_VERSION;
	buf->data[MQTL_TYPE_IDX] = type;
	buf->data[MQTL_FLAGS_IDX] =
		(qos & MQTL_FLAG_QOS_MASK) | extra_flags | (use_topic_id ? MQTL_FLAG_TOPIC_ID : 0);
	buf->data[MQTL_RESERVED_IDX] = 0;
	sys_put_le16(packet_id, &buf->data[MQTL_PACKET_ID_IDX]);
	sys_put_le16(wire_topic_len, &buf->data[MQTL_TOPIC_LEN_IDX]);
	sys_put_le16(payload_len, &buf->data[MQTL_PAYLOAD_LEN_IDX]);

	if (use_topic_id) {
		sys_put_le16(topic_id, &buf->data[MQTL_HEADER_LEN]);
	} else if (topic_len > 0) {
		memcpy(&buf->data[MQTL_HEADER_LEN], topic, topic_len);
	}
	if (payload_len > 0) {
		memcpy(&buf->data[MQTL_HEADER_LEN + wire_topic_len], payload, payload_len);
	}

	return interface->api->send(interface, buf);
}

static int send_ack(struct AresProtocol *protocol, enum mqttlite_frame_type type,
		    uint16_t packet_id)
{
	return send_frame(protocol, type, ARES_MQTTLITE_QOS0, 0, packet_id, false, 0, NULL, 0, NULL,
			  0);
}

static void schedule_retry(struct ares_mqttlite_protocol_data *data)
{
	k_work_schedule(&data->retry_work, K_MSEC(CONFIG_ARES_MQTTLITE_RETRY_INTERVAL_MS));
}

static void dispatch_publish(struct AresProtocol *protocol, const char *topic, bool use_topic_id,
			     uint16_t topic_id, const uint8_t *payload, uint16_t payload_len,
			     enum ares_mqttlite_qos qos)
{
	struct ares_mqttlite_protocol_data *data = protocol->priv_data;
	ares_mqttlite_msg_cb_t callbacks[CONFIG_ARES_MQTTLITE_MAX_SUBSCRIPTIONS];
	void *user_data[CONFIG_ARES_MQTTLITE_MAX_SUBSCRIPTIONS];
	int count = 0;

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_SUBSCRIPTIONS; i++) {
		if (data->subscriptions[i].cb == NULL) {
			continue;
		}
		if (data->subscriptions[i].use_topic_id) {
			if (!use_topic_id || data->subscriptions[i].topic_id != topic_id) {
				continue;
			}
		} else if (!topic_matches(data->subscriptions[i].topic_filter, topic)) {
			continue;
		}
		{
			callbacks[count] = data->subscriptions[i].cb;
			user_data[count] = data->subscriptions[i].user_data;
			count++;
		}
	}
	k_mutex_unlock(&data->lock);

	for (int i = 0; i < count; i++) {
		callbacks[i](protocol, topic, payload, payload_len, qos, user_data[i]);
	}
}

static void clear_tx(struct ares_mqttlite_tx_inflight *tx)
{
	memset(tx, 0, sizeof(*tx));
	tx->state = ARES_MQTTLITE_TX_UNUSED;
}

static void complete_tx(struct AresProtocol *protocol, uint16_t packet_id, int status)
{
	struct ares_mqttlite_protocol_data *data = protocol->priv_data;
	ares_mqttlite_publish_cb_t cb = NULL;
	void *user_data = NULL;

	k_mutex_lock(&data->lock, K_FOREVER);
	struct ares_mqttlite_tx_inflight *tx = find_tx_locked(data, packet_id);
	if (tx != NULL) {
		cb = tx->cb;
		user_data = tx->user_data;
		clear_tx(tx);
	}
	k_mutex_unlock(&data->lock);

	if (cb != NULL) {
		cb(protocol, packet_id, status, user_data);
	}
}

static void handle_publish(struct AresProtocol *protocol, const uint8_t *frame)
{
	struct ares_mqttlite_protocol_data *data = protocol->priv_data;
	uint16_t packet_id = frame_packet_id(frame);
	uint16_t topic_len = frame_topic_len(frame);
	uint16_t payload_len = frame_payload_len(frame);
	enum ares_mqttlite_qos qos = frame_qos(frame);
	bool use_topic_id = frame_uses_topic_id(frame);
	uint16_t topic_id = 0;
	char topic[CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN + 1];
	const uint8_t *payload = &frame[MQTL_HEADER_LEN + topic_len];

	topic[0] = '\0';

	if (!valid_qos(qos) || topic_len == 0 ||
	    payload_len > CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE) {
		LOG_WRN("%s rejected malformed publish", data->name);
		return;
	}
	if (use_topic_id && topic_len != MQTL_TOPIC_ID_LEN) {
		LOG_WRN("%s rejected malformed topic-id publish", data->name);
		return;
	}
	if (!use_topic_id && topic_len > CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN) {
		LOG_WRN("%s rejected oversized topic publish", data->name);
		return;
	}
	if (qos != ARES_MQTTLITE_QOS0 && packet_id == 0) {
		LOG_WRN("%s rejected QoS publish without packet id", data->name);
		return;
	}

	if (use_topic_id) {
		const char *registered_topic;

		topic_id = sys_get_le16(&frame[MQTL_HEADER_LEN]);
		k_mutex_lock(&data->lock, K_FOREVER);
		registered_topic = find_topic_name_locked(data, topic_id);
		if (registered_topic != NULL) {
			strncpy(topic, registered_topic, sizeof(topic) - 1);
			topic[sizeof(topic) - 1] = '\0';
		}
		k_mutex_unlock(&data->lock);
	} else {
		memcpy(topic, &frame[MQTL_HEADER_LEN], topic_len);
		topic[topic_len] = '\0';
	}

	if (qos == ARES_MQTTLITE_QOS0) {
		dispatch_publish(protocol, topic, use_topic_id, topic_id, payload, payload_len,
				 qos);
		return;
	}

	if (qos == ARES_MQTTLITE_QOS1) {
		dispatch_publish(protocol, topic, use_topic_id, topic_id, payload, payload_len,
				 qos);
		(void)send_ack(protocol, MQTL_FRAME_PUBACK, packet_id);
		return;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	struct ares_mqttlite_rx_qos2 *rx = find_rx_qos2_locked(data, packet_id);
	if (rx == NULL) {
		rx = alloc_rx_qos2_locked(data);
		if (rx != NULL) {
			rx->used = true;
			rx->packet_id = packet_id;
			rx->use_topic_id = use_topic_id;
			rx->topic_id = topic_id;
			rx->topic_len = strlen(topic);
			rx->payload_len = payload_len;
			memcpy(rx->topic, topic, rx->topic_len + 1);
			memcpy(rx->payload, payload, payload_len);
		}
	}
	k_mutex_unlock(&data->lock);

	if (rx == NULL) {
		LOG_WRN("%s has no room for QoS2 packet %u", data->name, packet_id);
		return;
	}

	(void)send_ack(protocol, MQTL_FRAME_PUBREC, packet_id);
}

static void handle_pubrel(struct AresProtocol *protocol, uint16_t packet_id)
{
	struct ares_mqttlite_protocol_data *data = protocol->priv_data;
	char topic[CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN + 1];
	uint8_t payload[CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE];
	uint16_t payload_len = 0;
	bool use_topic_id = false;
	uint16_t topic_id = 0;
	bool found = false;

	k_mutex_lock(&data->lock, K_FOREVER);
	struct ares_mqttlite_rx_qos2 *rx = find_rx_qos2_locked(data, packet_id);
	if (rx != NULL) {
		memcpy(topic, rx->topic, rx->topic_len + 1);
		memcpy(payload, rx->payload, rx->payload_len);
		payload_len = rx->payload_len;
		use_topic_id = rx->use_topic_id;
		topic_id = rx->topic_id;
		memset(rx, 0, sizeof(*rx));
		found = true;
	}
	k_mutex_unlock(&data->lock);

	if (found) {
		dispatch_publish(protocol, topic, use_topic_id, topic_id, payload, payload_len,
				 ARES_MQTTLITE_QOS2);
	}
	(void)send_ack(protocol, MQTL_FRAME_PUBCOMP, packet_id);
}

static void handle_ack_frame(struct AresProtocol *protocol, enum mqttlite_frame_type type,
			     uint16_t packet_id)
{
	struct ares_mqttlite_protocol_data *data = protocol->priv_data;

	if (packet_id == 0) {
		return;
	}

	if (type == MQTL_FRAME_PUBACK || type == MQTL_FRAME_PUBCOMP) {
		complete_tx(protocol, packet_id, ARES_MQTTLITE_PUBLISH_ACKED);
		return;
	}

	if (type == MQTL_FRAME_PUBREC) {
		bool send_pubrel = false;

		k_mutex_lock(&data->lock, K_FOREVER);
		struct ares_mqttlite_tx_inflight *tx = find_tx_locked(data, packet_id);
		if (tx != NULL && tx->state == ARES_MQTTLITE_TX_WAIT_PUBREC) {
			tx->state = ARES_MQTTLITE_TX_WAIT_PUBCOMP;
			tx->retries = 0;
			tx->last_send_ms = k_uptime_get();
			send_pubrel = true;
			schedule_retry(data);
		}
		k_mutex_unlock(&data->lock);

		if (send_pubrel) {
			(void)send_ack(protocol, MQTL_FRAME_PUBREL, packet_id);
		}
	}
}

static void process_frame(struct AresProtocol *protocol, const uint8_t *frame, uint16_t len)
{
	uint16_t topic_len;
	uint16_t payload_len;

	if (len < MQTL_HEADER_LEN || sys_get_le16(&frame[MQTL_MAGIC_IDX]) != MQTL_MAGIC ||
	    frame[MQTL_VERSION_IDX] != MQTL_VERSION) {
		return;
	}

	topic_len = frame_topic_len(frame);
	payload_len = frame_payload_len(frame);
	if ((uint32_t)MQTL_HEADER_LEN + topic_len + payload_len != len) {
		return;
	}

	switch (frame[MQTL_TYPE_IDX]) {
	case MQTL_FRAME_PUBLISH:
		handle_publish(protocol, frame);
		break;
	case MQTL_FRAME_PUBACK:
	case MQTL_FRAME_PUBREC:
	case MQTL_FRAME_PUBCOMP:
		handle_ack_frame(protocol, frame[MQTL_TYPE_IDX], frame_packet_id(frame));
		break;
	case MQTL_FRAME_PUBREL:
		handle_pubrel(protocol, frame_packet_id(frame));
		break;
	case MQTL_FRAME_PING:
		(void)send_ack(protocol, MQTL_FRAME_PONG, 0);
		break;
	case MQTL_FRAME_PONG:
		break;
	default:
		break;
	}
}

static void retry_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct ares_mqttlite_protocol_data *data =
		CONTAINER_OF(dwork, struct ares_mqttlite_protocol_data, retry_work);
	struct AresProtocol *protocol = data->protocol;
	int64_t now = k_uptime_get();
	bool has_pending = false;

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_INFLIGHT; i++) {
		struct ares_mqttlite_tx_inflight *tx = &data->tx[i];
		enum ares_mqttlite_tx_state state;
		enum ares_mqttlite_qos qos;
		bool use_topic_id;
		uint16_t packet_id;
		uint16_t topic_id;
		uint16_t topic_len;
		uint16_t payload_len;
		char topic[CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN + 1];
		uint8_t payload[CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE];

		if (tx->state == ARES_MQTTLITE_TX_UNUSED) {
			continue;
		}
		has_pending = true;
		if (now - tx->last_send_ms < CONFIG_ARES_MQTTLITE_RETRY_INTERVAL_MS) {
			continue;
		}
		if (tx->retries >= CONFIG_ARES_MQTTLITE_MAX_RETRIES) {
			uint16_t packet_id = tx->packet_id;
			ares_mqttlite_publish_cb_t cb = tx->cb;
			void *user_data = tx->user_data;

			clear_tx(tx);
			k_mutex_unlock(&data->lock);
			if (cb != NULL) {
				cb(protocol, packet_id, ARES_MQTTLITE_PUBLISH_TIMEOUT, user_data);
			}
			k_mutex_lock(&data->lock, K_FOREVER);
			has_pending = false;
			for (int j = 0; j < CONFIG_ARES_MQTTLITE_MAX_INFLIGHT; j++) {
				if (data->tx[j].state != ARES_MQTTLITE_TX_UNUSED) {
					has_pending = true;
					break;
				}
			}
			continue;
		}

		state = tx->state;
		qos = tx->qos;
		use_topic_id = tx->use_topic_id;
		packet_id = tx->packet_id;
		topic_id = tx->topic_id;
		topic_len = tx->topic_len;
		payload_len = tx->payload_len;
		memcpy(topic, tx->topic, topic_len + 1);
		memcpy(payload, tx->payload, payload_len);
		tx->retries++;
		tx->last_send_ms = now;
		k_mutex_unlock(&data->lock);
		if (state == ARES_MQTTLITE_TX_WAIT_PUBCOMP) {
			(void)send_ack(protocol, MQTL_FRAME_PUBREL, packet_id);
		} else {
			(void)send_frame(protocol, MQTL_FRAME_PUBLISH, qos, MQTL_FLAG_DUP,
					 packet_id, use_topic_id, topic_id, topic, topic_len,
					 payload, payload_len);
		}
		k_mutex_lock(&data->lock, K_FOREVER);
	}
	k_mutex_unlock(&data->lock);

	if (has_pending) {
		schedule_retry(data);
	}
}

int ares_mqttlite_register_topic(struct AresProtocol *protocol, uint16_t topic_id,
				 const char *topic)
{
	struct ares_mqttlite_protocol_data *data;
	size_t topic_len;
	int empty = -1;

	if (protocol == NULL || topic_id == 0 || topic == NULL) {
		return -EINVAL;
	}

	data = protocol->priv_data;
	topic_len = strlen(topic);
	if (topic_len == 0 || topic_len > CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_TOPICS; i++) {
		if (!data->topics[i].used && empty < 0) {
			empty = i;
			continue;
		}
		if (data->topics[i].used && data->topics[i].topic_id == topic_id) {
			memcpy(data->topics[i].topic, topic, topic_len + 1);
			k_mutex_unlock(&data->lock);
			return 0;
		}
	}
	if (empty < 0) {
		k_mutex_unlock(&data->lock);
		return -ENOMEM;
	}

	data->topics[empty].used = true;
	data->topics[empty].topic_id = topic_id;
	memcpy(data->topics[empty].topic, topic, topic_len + 1);
	k_mutex_unlock(&data->lock);

	return 0;
}

int ares_mqttlite_subscribe(struct AresProtocol *protocol, const char *topic_filter,
			    ares_mqttlite_msg_cb_t cb, void *user_data)
{
	struct ares_mqttlite_protocol_data *data;
	size_t topic_len;
	int empty = -1;

	if (protocol == NULL || topic_filter == NULL || cb == NULL) {
		return -EINVAL;
	}

	data = protocol->priv_data;
	topic_len = strlen(topic_filter);
	if (topic_len == 0 || topic_len > CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_SUBSCRIPTIONS; i++) {
		if (data->subscriptions[i].cb == NULL && empty < 0) {
			empty = i;
			continue;
		}
		if (!data->subscriptions[i].use_topic_id &&
		    strcmp(data->subscriptions[i].topic_filter, topic_filter) == 0) {
			data->subscriptions[i].cb = cb;
			data->subscriptions[i].user_data = user_data;
			k_mutex_unlock(&data->lock);
			return 0;
		}
	}
	if (empty < 0) {
		k_mutex_unlock(&data->lock);
		return -ENOMEM;
	}

	memcpy(data->subscriptions[empty].topic_filter, topic_filter, topic_len + 1);
	data->subscriptions[empty].cb = cb;
	data->subscriptions[empty].user_data = user_data;
	k_mutex_unlock(&data->lock);

	return 0;
}

int ares_mqttlite_subscribe_id(struct AresProtocol *protocol, uint16_t topic_id,
			       ares_mqttlite_msg_cb_t cb, void *user_data)
{
	struct ares_mqttlite_protocol_data *data;
	int empty = -1;

	if (protocol == NULL || topic_id == 0 || cb == NULL) {
		return -EINVAL;
	}

	data = protocol->priv_data;
	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_SUBSCRIPTIONS; i++) {
		if (data->subscriptions[i].cb == NULL && empty < 0) {
			empty = i;
			continue;
		}
		if (data->subscriptions[i].use_topic_id &&
		    data->subscriptions[i].topic_id == topic_id) {
			data->subscriptions[i].cb = cb;
			data->subscriptions[i].user_data = user_data;
			k_mutex_unlock(&data->lock);
			return 0;
		}
	}
	if (empty < 0) {
		k_mutex_unlock(&data->lock);
		return -ENOMEM;
	}

	data->subscriptions[empty].use_topic_id = true;
	data->subscriptions[empty].topic_id = topic_id;
	data->subscriptions[empty].topic_filter[0] = '\0';
	data->subscriptions[empty].cb = cb;
	data->subscriptions[empty].user_data = user_data;
	k_mutex_unlock(&data->lock);

	return 0;
}

int ares_mqttlite_unsubscribe(struct AresProtocol *protocol, const char *topic_filter)
{
	struct ares_mqttlite_protocol_data *data;

	if (protocol == NULL || topic_filter == NULL) {
		return -EINVAL;
	}

	data = protocol->priv_data;
	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_SUBSCRIPTIONS; i++) {
		if (!data->subscriptions[i].use_topic_id &&
		    strcmp(data->subscriptions[i].topic_filter, topic_filter) == 0) {
			memset(&data->subscriptions[i], 0, sizeof(data->subscriptions[i]));
			k_mutex_unlock(&data->lock);
			return 0;
		}
	}
	k_mutex_unlock(&data->lock);

	return -ENOENT;
}

int ares_mqttlite_unsubscribe_id(struct AresProtocol *protocol, uint16_t topic_id)
{
	struct ares_mqttlite_protocol_data *data;

	if (protocol == NULL || topic_id == 0) {
		return -EINVAL;
	}

	data = protocol->priv_data;
	k_mutex_lock(&data->lock, K_FOREVER);
	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_SUBSCRIPTIONS; i++) {
		if (data->subscriptions[i].use_topic_id &&
		    data->subscriptions[i].topic_id == topic_id) {
			memset(&data->subscriptions[i], 0, sizeof(data->subscriptions[i]));
			k_mutex_unlock(&data->lock);
			return 0;
		}
	}
	k_mutex_unlock(&data->lock);

	return -ENOENT;
}

static int publish_common(struct AresProtocol *protocol, bool use_topic_id, uint16_t topic_id,
			  const char *topic, uint16_t topic_len, const uint8_t *payload,
			  uint16_t payload_len, enum ares_mqttlite_qos qos,
			  ares_mqttlite_publish_cb_t cb, void *user_data)
{
	struct ares_mqttlite_protocol_data *data;
	uint16_t packet_id = 0;
	int ret;

	if (protocol == NULL || !valid_qos(qos)) {
		return -EINVAL;
	}
	if (payload == NULL && payload_len > 0) {
		return -EINVAL;
	}

	data = protocol->priv_data;
	if (use_topic_id) {
		if (topic_id == 0) {
			return -EINVAL;
		}
		topic_len = 0;
		topic = "";
	} else if (topic == NULL || topic_len == 0 ||
		   topic_len > CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN) {
		return -EINVAL;
	}
	if (payload_len > CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE) {
		return -EINVAL;
	}

	if (qos != ARES_MQTTLITE_QOS0) {
		k_mutex_lock(&data->lock, K_FOREVER);
		struct ares_mqttlite_tx_inflight *tx = alloc_tx_locked(data);
		if (tx == NULL) {
			k_mutex_unlock(&data->lock);
			return -ENOMEM;
		}
		packet_id = alloc_packet_id_locked(data);
		tx->state = qos == ARES_MQTTLITE_QOS1 ? ARES_MQTTLITE_TX_WAIT_PUBACK
						      : ARES_MQTTLITE_TX_WAIT_PUBREC;
		tx->packet_id = packet_id;
		tx->qos = qos;
		tx->retries = 0;
		tx->last_send_ms = k_uptime_get();
		tx->cb = cb;
		tx->user_data = user_data;
		tx->use_topic_id = use_topic_id;
		tx->topic_id = topic_id;
		tx->topic_len = topic_len;
		tx->payload_len = payload_len;
		memcpy(tx->topic, topic, topic_len + 1);
		if (payload_len > 0) {
			memcpy(tx->payload, payload, payload_len);
		}
		k_mutex_unlock(&data->lock);
	}

	ret = send_frame(protocol, MQTL_FRAME_PUBLISH, qos, 0, packet_id, use_topic_id, topic_id,
			 topic, topic_len, payload, payload_len);
	if (ret != 0 && qos != ARES_MQTTLITE_QOS0) {
		complete_tx(protocol, packet_id, ret);
		return ret;
	}

	if (qos != ARES_MQTTLITE_QOS0) {
		schedule_retry(data);
		return packet_id;
	}

	return ret;
}

int ares_mqttlite_publish(struct AresProtocol *protocol, const char *topic, const uint8_t *payload,
			  uint16_t payload_len, enum ares_mqttlite_qos qos,
			  ares_mqttlite_publish_cb_t cb, void *user_data)
{
	if (topic == NULL) {
		return -EINVAL;
	}

	return publish_common(protocol, false, 0, topic, strlen(topic), payload, payload_len, qos,
			      cb, user_data);
}

int ares_mqttlite_publish_id(struct AresProtocol *protocol, uint16_t topic_id,
			     const uint8_t *payload, uint16_t payload_len,
			     enum ares_mqttlite_qos qos, ares_mqttlite_publish_cb_t cb,
			     void *user_data)
{
	return publish_common(protocol, true, topic_id, NULL, 0, payload, payload_len, qos, cb,
			      user_data);
}

int ares_mqttlite_publish_prepare_id(struct AresProtocol *protocol, uint16_t topic_id,
				     uint16_t payload_len, struct ares_mqttlite_publish_buffer *pub)
{
	struct AresInterface *interface;
	size_t frame_len = MQTL_HEADER_LEN + MQTL_TOPIC_ID_LEN + payload_len;

	if (protocol == NULL || topic_id == 0 || pub == NULL ||
	    payload_len > CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE ||
	    frame_len > CONFIG_ARES_MQTTLITE_MAX_FRAME_SIZE) {
		return -EINVAL;
	}

	interface = protocol->interface;
	if (interface == NULL || interface->api == NULL || interface->api->send == NULL ||
	    interface->api->alloc_buf == NULL) {
		return -ENODEV;
	}

	memset(pub, 0, sizeof(*pub));
	pub->buf = interface->api->alloc_buf(interface);
	if (pub->buf == NULL) {
		return -ENOMEM;
	}
	if (net_buf_tailroom(pub->buf) < frame_len) {
		net_buf_unref(pub->buf);
		memset(pub, 0, sizeof(*pub));
		return -EMSGSIZE;
	}

	net_buf_add(pub->buf, frame_len);
	sys_put_le16(MQTL_MAGIC, &pub->buf->data[MQTL_MAGIC_IDX]);
	pub->buf->data[MQTL_VERSION_IDX] = MQTL_VERSION;
	pub->buf->data[MQTL_TYPE_IDX] = MQTL_FRAME_PUBLISH;
	pub->buf->data[MQTL_FLAGS_IDX] = ARES_MQTTLITE_QOS0 | MQTL_FLAG_TOPIC_ID;
	pub->buf->data[MQTL_RESERVED_IDX] = 0;
	sys_put_le16(0, &pub->buf->data[MQTL_PACKET_ID_IDX]);
	sys_put_le16(MQTL_TOPIC_ID_LEN, &pub->buf->data[MQTL_TOPIC_LEN_IDX]);
	sys_put_le16(payload_len, &pub->buf->data[MQTL_PAYLOAD_LEN_IDX]);
	sys_put_le16(topic_id, &pub->buf->data[MQTL_HEADER_LEN]);
	pub->payload = &pub->buf->data[MQTL_HEADER_LEN + MQTL_TOPIC_ID_LEN];
	pub->payload_len = payload_len;

	return 0;
}

int ares_mqttlite_publish_commit(struct AresProtocol *protocol,
				 struct ares_mqttlite_publish_buffer *pub)
{
	struct AresInterface *interface;
	struct net_buf *buf;

	if (protocol == NULL || pub == NULL || pub->buf == NULL) {
		return -EINVAL;
	}

	interface = protocol->interface;
	if (interface == NULL || interface->api == NULL || interface->api->send == NULL) {
		ares_mqttlite_publish_abort(pub);
		return -ENODEV;
	}

	buf = pub->buf;
	memset(pub, 0, sizeof(*pub));
	return interface->api->send(interface, buf);
}

void ares_mqttlite_publish_abort(struct ares_mqttlite_publish_buffer *pub)
{
	if (pub != NULL && pub->buf != NULL) {
		net_buf_unref(pub->buf);
		memset(pub, 0, sizeof(*pub));
	}
}

int ares_mqttlite_ping(struct AresProtocol *protocol)
{
	return send_ack(protocol, MQTL_FRAME_PING, 0);
}

void ares_mqttlite_protocol_handle_byte(struct AresProtocol *protocol, uint8_t byte)
{
	struct ares_mqttlite_protocol_data *data = protocol->priv_data;

	if (data->rx_pos >= sizeof(data->rx_buf)) {
		data->rx_pos = 0;
		data->rx_expected = 0;
	}

	data->rx_buf[data->rx_pos++] = byte;

	if (data->rx_pos == MQTL_HEADER_LEN) {
		if (sys_get_le16(&data->rx_buf[MQTL_MAGIC_IDX]) != MQTL_MAGIC ||
		    data->rx_buf[MQTL_VERSION_IDX] != MQTL_VERSION) {
			memmove(data->rx_buf, &data->rx_buf[1], data->rx_pos - 1);
			data->rx_pos--;
			return;
		}
		data->rx_expected = MQTL_HEADER_LEN + frame_topic_len(data->rx_buf) +
				    frame_payload_len(data->rx_buf);
		if (data->rx_expected > sizeof(data->rx_buf)) {
			data->rx_pos = 0;
			data->rx_expected = 0;
			return;
		}
	}

	if (data->rx_expected != 0 && data->rx_pos >= data->rx_expected) {
		uint8_t frame[CONFIG_ARES_MQTTLITE_MAX_FRAME_SIZE];
		uint16_t frame_len = data->rx_expected;

		memcpy(frame, data->rx_buf, frame_len);
		data->rx_pos = 0;
		data->rx_expected = 0;
		process_frame(protocol, frame, frame_len);
	}
}

void ares_mqttlite_protocol_handle(struct AresProtocol *protocol, struct net_buf *buf)
{
	struct ares_mqttlite_protocol_data *data = protocol->priv_data;
	uint16_t pos = 0;

	if (data->rx_pos == 0) {
		while (buf->len - pos >= MQTL_HEADER_LEN) {
			uint16_t frame_len;

			if (sys_get_le16(&buf->data[pos + MQTL_MAGIC_IDX]) != MQTL_MAGIC ||
			    buf->data[pos + MQTL_VERSION_IDX] != MQTL_VERSION) {
				break;
			}

			frame_len = MQTL_HEADER_LEN +
				    sys_get_le16(&buf->data[pos + MQTL_TOPIC_LEN_IDX]) +
				    sys_get_le16(&buf->data[pos + MQTL_PAYLOAD_LEN_IDX]);
			if (frame_len > CONFIG_ARES_MQTTLITE_MAX_FRAME_SIZE) {
				pos = buf->len;
				break;
			}
			if (frame_len > buf->len - pos) {
				break;
			}

			process_frame(protocol, &buf->data[pos], frame_len);
			pos += frame_len;
		}
	}

	for (uint16_t i = pos; i < buf->len; i++) {
		ares_mqttlite_protocol_handle_byte(protocol, buf->data[i]);
	}
}

void ares_mqttlite_protocol_event(struct AresProtocol *protocol, enum AresProtocolEvent event)
{
	struct ares_mqttlite_protocol_data *data = protocol->priv_data;

	k_mutex_lock(&data->lock, K_FOREVER);
	data->online = event == ARES_PROTOCOL_EVENT_CONNECTED;
	if (!data->online) {
		(void)k_work_cancel_delayable(&data->retry_work);
		for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_INFLIGHT; i++) {
			clear_tx(&data->tx[i]);
			memset(&data->rx_qos2[i], 0, sizeof(data->rx_qos2[i]));
		}
		data->rx_pos = 0;
		data->rx_expected = 0;
	}
	k_mutex_unlock(&data->lock);
}

int ares_mqttlite_protocol_init(struct AresProtocol *protocol)
{
	struct ares_mqttlite_protocol_data *data;

	if (protocol == NULL || protocol->priv_data == NULL) {
		return -EINVAL;
	}

	data = protocol->priv_data;
	k_mutex_init(&data->lock);
	k_work_init_delayable(&data->retry_work, retry_work_handler);
	data->protocol = protocol;
	data->next_packet_id = 1;
	data->rx_pos = 0;
	data->rx_expected = 0;
	data->online = false;
	memset(data->topics, 0, sizeof(data->topics));
	memset(data->subscriptions, 0, sizeof(data->subscriptions));
	memset(data->rx_qos2, 0, sizeof(data->rx_qos2));

	for (int i = 0; i < CONFIG_ARES_MQTTLITE_MAX_INFLIGHT; i++) {
		clear_tx(&data->tx[i]);
	}

	LOG_INF("%s MQTT-like protocol init", data->name);
	return 0;
}
