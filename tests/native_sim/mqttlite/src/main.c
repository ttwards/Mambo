/*
 * Copyright (c) 2026 ttwards
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <limits.h>

#include <zephyr/net_buf.h>
#include <zephyr/ztest.h>

#include <ares/ares_comm.h>
#include <ares/interface/ares_interface.h>
#include <ares/protocol/mqttlite/mqttlite_protocol.h>

NET_BUF_POOL_DEFINE(fake_pool, 16, 512, 0, NULL);

struct fake_link {
	struct AresInterface *peer;
};

static int fake_init(struct AresInterface *interface)
{
	ARG_UNUSED(interface);
	return 0;
}

static struct net_buf *fake_alloc_buf(struct AresInterface *interface)
{
	ARG_UNUSED(interface);
	return net_buf_alloc(&fake_pool, K_NO_WAIT);
}

static int fake_send(struct AresInterface *interface, struct net_buf *buf)
{
	struct fake_link *link = interface->priv_data;

	zassert_not_null(link->peer, "fake link peer must be set");
	zassert_not_null(link->peer->protocol, "fake link peer protocol must be bound");

	link->peer->protocol->api->handle(link->peer->protocol, buf);
	net_buf_unref(buf);

	return 0;
}

static const struct AresInterfaceAPI fake_api = {
	.init = fake_init,
	.send = fake_send,
	.alloc_buf = fake_alloc_buf,
};

static struct fake_link client_link;
static struct fake_link server_link;

static struct AresInterface client_if = {
	.name = "client",
	.api = &fake_api,
	.priv_data = &client_link,
};

static struct AresInterface server_if = {
	.name = "server",
	.api = &fake_api,
	.priv_data = &server_link,
};

ARES_MQTTLITE_PROTOCOL_DEFINE(client_protocol);
ARES_MQTTLITE_PROTOCOL_DEFINE(server_protocol);

static int rx_count;
static enum ares_mqttlite_qos last_qos;
static char last_topic[CONFIG_ARES_MQTTLITE_MAX_TOPIC_LEN + 1];
static char last_payload[CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE + 1];
static int publish_status;
static uint16_t publish_packet_id;

static void reset_observed(void)
{
	rx_count = 0;
	last_qos = ARES_MQTTLITE_QOS0;
	memset(last_topic, 0, sizeof(last_topic));
	memset(last_payload, 0, sizeof(last_payload));
	publish_status = INT_MIN;
	publish_packet_id = 0;
}

static void message_cb(struct AresProtocol *protocol, const char *topic, const uint8_t *payload,
		       uint16_t payload_len, enum ares_mqttlite_qos qos, void *user_data)
{
	ARG_UNUSED(protocol);
	ARG_UNUSED(user_data);

	rx_count++;
	last_qos = qos;
	strncpy(last_topic, topic, sizeof(last_topic) - 1);
	last_topic[sizeof(last_topic) - 1] = '\0';
	memcpy(last_payload, payload, payload_len);
	last_payload[payload_len] = '\0';
}

static void publish_cb(struct AresProtocol *protocol, uint16_t packet_id, int status,
		       void *user_data)
{
	ARG_UNUSED(protocol);
	ARG_UNUSED(user_data);

	publish_packet_id = packet_id;
	publish_status = status;
}

static void bind_pair(void)
{
	client_link.peer = &server_if;
	server_link.peer = &client_if;

	zassert_ok(ares_bind_interface(&client_if, &client_protocol));
	zassert_ok(ares_bind_interface(&server_if, &server_protocol));
	client_protocol.api->event(&client_protocol, ARES_PROTOCOL_EVENT_CONNECTED);
	server_protocol.api->event(&server_protocol, ARES_PROTOCOL_EVENT_CONNECTED);
}

static void *suite_setup(void)
{
	bind_pair();
	return NULL;
}

ZTEST(ares_mqttlite, test_qos0_publish_delivers_to_subscriber)
{
	const uint8_t payload[] = "ready";

	reset_observed();

	zassert_ok(ares_mqttlite_subscribe(&server_protocol, "robot/state", message_cb, NULL));
	zassert_ok(ares_mqttlite_publish(&client_protocol, "robot/state", payload,
					 sizeof(payload) - 1, ARES_MQTTLITE_QOS0, NULL, NULL));

	zassert_equal(rx_count, 1, "rx_count=%d", rx_count);
	zassert_equal(last_qos, ARES_MQTTLITE_QOS0);
	zassert_mem_equal(last_payload, "ready", sizeof("ready"));
}

ZTEST(ares_mqttlite, test_qos1_publish_gets_puback)
{
	const uint8_t payload[] = "armed";
	int packet_id;

	reset_observed();

	zassert_ok(ares_mqttlite_subscribe(&server_protocol, "robot/state", message_cb, NULL));
	packet_id =
		ares_mqttlite_publish(&client_protocol, "robot/state", payload, sizeof(payload) - 1,
				      ARES_MQTTLITE_QOS1, publish_cb, NULL);

	zassert_true(packet_id > 0, "QoS1 publish should return a packet id");
	zassert_equal(rx_count, 1, "rx_count=%d", rx_count);
	zassert_equal(last_qos, ARES_MQTTLITE_QOS1);
	zassert_equal(publish_packet_id, packet_id);
	zassert_equal(publish_status, ARES_MQTTLITE_PUBLISH_ACKED);
}

ZTEST(ares_mqttlite, test_qos2_publish_completes_exactly_once)
{
	const uint8_t payload[] = "shoot";
	int packet_id;

	reset_observed();

	zassert_ok(ares_mqttlite_subscribe(&server_protocol, "robot/command", message_cb, NULL));
	packet_id =
		ares_mqttlite_publish(&client_protocol, "robot/command", payload,
				      sizeof(payload) - 1, ARES_MQTTLITE_QOS2, publish_cb, NULL);

	zassert_true(packet_id > 0, "QoS2 publish should return a packet id");
	zassert_equal(rx_count, 1, "rx_count=%d", rx_count);
	zassert_equal(last_qos, ARES_MQTTLITE_QOS2);
	zassert_mem_equal(last_payload, "shoot", sizeof("shoot"));
	zassert_equal(publish_packet_id, packet_id);
	zassert_equal(publish_status, ARES_MQTTLITE_PUBLISH_ACKED);
}

ZTEST(ares_mqttlite, test_topic_wildcards_match)
{
	const uint8_t payload[] = "42";

	reset_observed();

	zassert_ok(ares_mqttlite_subscribe(&server_protocol, "robot/+/rpm", message_cb, NULL));
	zassert_ok(ares_mqttlite_publish(&client_protocol, "robot/front/rpm", payload,
					 sizeof(payload) - 1, ARES_MQTTLITE_QOS0, NULL, NULL));

	zassert_equal(rx_count, 1);
	zassert_mem_equal(last_payload, "42", sizeof("42"));
}

ZTEST(ares_mqttlite, test_topic_id_qos1_publish_gets_puback)
{
	const uint8_t payload[] = "fast";
	int packet_id;

	reset_observed();

	zassert_ok(ares_mqttlite_register_topic(&server_protocol, 7, "robot/fast"));
	zassert_ok(ares_mqttlite_subscribe_id(&server_protocol, 7, message_cb, NULL));
	packet_id = ares_mqttlite_publish_id(&client_protocol, 7, payload, sizeof(payload) - 1,
					     ARES_MQTTLITE_QOS1, publish_cb, NULL);

	zassert_true(packet_id > 0, "QoS1 id publish should return a packet id");
	zassert_equal(rx_count, 1, "rx_count=%d", rx_count);
	zassert_equal(last_qos, ARES_MQTTLITE_QOS1);
	zassert_mem_equal(last_topic, "robot/fast", sizeof("robot/fast"));
	zassert_mem_equal(last_payload, "fast", sizeof("fast"));
	zassert_equal(publish_packet_id, packet_id);
	zassert_equal(publish_status, ARES_MQTTLITE_PUBLISH_ACKED);
}

ZTEST(ares_mqttlite, test_topic_id_qos0_prepare_commit_is_zero_copy)
{
	struct ares_mqttlite_publish_buffer pub;

	reset_observed();

	zassert_ok(ares_mqttlite_register_topic(&server_protocol, 8, "robot/zero"));
	zassert_ok(ares_mqttlite_subscribe_id(&server_protocol, 8, message_cb, NULL));
	zassert_ok(ares_mqttlite_publish_prepare_id(&client_protocol, 8, 4, &pub));
	zassert_not_null(pub.payload);
	memcpy(pub.payload, "zero", 4);
	zassert_ok(ares_mqttlite_publish_commit(&client_protocol, &pub));

	zassert_equal(rx_count, 1, "rx_count=%d", rx_count);
	zassert_equal(last_qos, ARES_MQTTLITE_QOS0);
	zassert_mem_equal(last_topic, "robot/zero", sizeof("robot/zero"));
	zassert_mem_equal(last_payload, "zero", sizeof("zero"));
}

ZTEST_SUITE(ares_mqttlite, NULL, suite_setup, NULL, NULL, NULL);
