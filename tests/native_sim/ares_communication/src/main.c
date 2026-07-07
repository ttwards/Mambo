/*
 * Copyright (c) 2026 ttwards
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/ztest.h>

#include <ares/ares_comm.h>
#include <ares/interface/ares_interface.h>
#include <ares/protocol/dual/dual_protocol.h>

NET_BUF_POOL_DEFINE(fake_buf_pool, 48, 320, 0, NULL);

struct fake_transport {
	struct AresInterface *peer;
	bool connected;
	bool hold_callback;
	bool fail_next_send;
	bool deliver_as_bytes;

	int init_calls;
	int send_calls;
	int send_with_callback_calls;
	int callback_calls;
	int delivered_frames;
	int last_callback_status;

	uint8_t last_frame[320];
	size_t last_len;

	struct net_buf *held_buf;
	ares_interface_tx_done_cb_t held_cb;
	void *held_user_data;
};

struct bind_observer {
	int order;
	int interface_init_order;
	int protocol_init_order;
	bool interface_saw_protocol;
	bool protocol_saw_interface;
};

static struct bind_observer bind_obs;

static void fake_transport_reset(struct fake_transport *transport)
{
	memset(transport, 0, sizeof(*transport));
}

static void fake_record_frame(struct fake_transport *transport, struct net_buf *buf)
{
	zassert_true(buf->len <= sizeof(transport->last_frame), "frame too large");
	memcpy(transport->last_frame, buf->data, buf->len);
	transport->last_len = buf->len;
}

static void fake_deliver_to_peer(struct fake_transport *transport, struct net_buf *buf)
{
	if (transport->peer == NULL || transport->peer->protocol == NULL) {
		return;
	}

	if (transport->deliver_as_bytes && transport->peer->protocol->api->handle_byte != NULL) {
		for (size_t i = 0; i < buf->len; i++) {
			transport->peer->protocol->api->handle_byte(transport->peer->protocol,
								    buf->data[i]);
		}
	} else if (transport->peer->protocol->api->handle != NULL) {
		struct net_buf *rx = net_buf_alloc(&fake_buf_pool, K_NO_WAIT);

		zassert_not_null(rx, "failed to allocate RX clone");
		net_buf_add_mem(rx, buf->data, buf->len);
		transport->peer->protocol->api->handle(transport->peer->protocol, rx);
		net_buf_unref(rx);
	}

	transport->delivered_frames++;
}

static void fake_complete_held_tx(struct AresInterface *interface, int status)
{
	struct fake_transport *transport = interface->priv_data;
	struct net_buf *buf = transport->held_buf;
	ares_interface_tx_done_cb_t cb = transport->held_cb;
	void *user_data = transport->held_user_data;

	zassert_not_null(buf, "no held TX buffer");
	transport->held_buf = NULL;
	transport->held_cb = NULL;
	transport->held_user_data = NULL;

	if (cb != NULL) {
		cb(interface, buf, status, user_data);
		transport->callback_calls++;
		transport->last_callback_status = status;
	}

	net_buf_unref(buf);
}

static int fake_interface_init(struct AresInterface *interface)
{
	struct fake_transport *transport = interface->priv_data;

	transport->init_calls++;
	transport->connected = true;
	return 0;
}

static int fake_send(struct AresInterface *interface, struct net_buf *buf)
{
	struct fake_transport *transport = interface->priv_data;

	transport->send_calls++;
	fake_record_frame(transport, buf);
	fake_deliver_to_peer(transport, buf);
	net_buf_unref(buf);
	return 0;
}

static int fake_send_with_callback(struct AresInterface *interface, struct net_buf *buf,
				   ares_interface_tx_done_cb_t cb, void *user_data)
{
	struct fake_transport *transport = interface->priv_data;

	transport->send_with_callback_calls++;
	fake_record_frame(transport, buf);

	if (transport->fail_next_send) {
		transport->fail_next_send = false;
		if (cb != NULL) {
			cb(interface, buf, -EIO, user_data);
			transport->callback_calls++;
			transport->last_callback_status = -EIO;
		}
		net_buf_unref(buf);
		return -EIO;
	}

	fake_deliver_to_peer(transport, buf);

	if (transport->hold_callback) {
		if (transport->held_buf != NULL) {
			if (cb != NULL) {
				cb(interface, buf, -EBUSY, user_data);
				transport->callback_calls++;
				transport->last_callback_status = -EBUSY;
			}
			net_buf_unref(buf);
			return -EBUSY;
		}

		transport->held_buf = buf;
		transport->held_cb = cb;
		transport->held_user_data = user_data;
		return 0;
	}

	if (cb != NULL) {
		cb(interface, buf, 0, user_data);
		transport->callback_calls++;
		transport->last_callback_status = 0;
	}
	net_buf_unref(buf);
	return 0;
}

static struct net_buf *fake_alloc_buf(struct AresInterface *interface)
{
	ARG_UNUSED(interface);

	return net_buf_alloc(&fake_buf_pool, K_NO_WAIT);
}

static struct net_buf *fake_alloc_buf_with_data(struct AresInterface *interface, void *data,
						size_t size)
{
	struct net_buf *buf = fake_alloc_buf(interface);

	if (buf != NULL) {
		net_buf_add_mem(buf, data, size);
	}

	return buf;
}

static bool fake_is_connected(struct AresInterface *interface)
{
	struct fake_transport *transport = interface->priv_data;

	return transport->connected;
}

static uint32_t fake_caps(struct AresInterface *interface)
{
	ARG_UNUSED(interface);

	return ARES_INTERFACE_CAP_STREAM | ARES_INTERFACE_CAP_TX_COMPLETE |
	       ARES_INTERFACE_CAP_TX_QUEUE;
}

static uint32_t fake_no_callback_caps(struct AresInterface *interface)
{
	ARG_UNUSED(interface);

	return ARES_INTERFACE_CAP_STREAM | ARES_INTERFACE_CAP_TX_QUEUE;
}

static size_t fake_mtu(struct AresInterface *interface)
{
	ARG_UNUSED(interface);

	return sizeof(((struct fake_transport *)0)->last_frame);
}

static const struct AresInterfaceAPI fake_interface_api = {
	.send = fake_send,
	.send_with_callback = fake_send_with_callback,
	.is_connected = fake_is_connected,
	.caps = fake_caps,
	.mtu = fake_mtu,
	.alloc_buf = fake_alloc_buf,
	.alloc_buf_with_data = fake_alloc_buf_with_data,
	.init = fake_interface_init,
};

static const struct AresInterfaceAPI fake_no_callback_api = {
	.send = fake_send,
	.is_connected = fake_is_connected,
	.caps = fake_no_callback_caps,
	.mtu = fake_mtu,
	.alloc_buf = fake_alloc_buf,
	.alloc_buf_with_data = fake_alloc_buf_with_data,
	.init = fake_interface_init,
};

static int observed_interface_init(struct AresInterface *interface)
{
	bind_obs.interface_init_order = ++bind_obs.order;
	bind_obs.interface_saw_protocol = interface->protocol != NULL;
	return 0;
}

static int observed_protocol_init(struct AresProtocol *protocol)
{
	bind_obs.protocol_init_order = ++bind_obs.order;
	bind_obs.protocol_saw_interface = protocol->interface != NULL;
	return 0;
}

static const struct AresInterfaceAPI observed_interface_api = {
	.init = observed_interface_init,
};

static const struct AresProtocolAPI observed_protocol_api = {
	.init = observed_protocol_init,
};

DUAL_PROPOSE_PROTOCOL_DEFINE(sync_protocol);
DUAL_PROPOSE_PROTOCOL_DEFINE(no_callback_protocol);
DUAL_PROPOSE_PROTOCOL_DEFINE(client_protocol);
DUAL_PROPOSE_PROTOCOL_DEFINE(server_protocol);
DUAL_PROPOSE_PROTOCOL_DEFINE(disconnect_protocol);

static bool sync_protocol_ready;
static bool no_callback_protocol_ready;
static bool client_protocol_ready;
static bool server_protocol_ready;
static bool disconnect_protocol_ready;

static void reset_dual_protocol(struct AresProtocol *protocol, const char *name, bool *ready)
{
	struct dual_protocol_data *data = protocol->priv_data;

	if (*ready) {
		k_timer_stop(&data->heart_beat_timer);
	}

	memset(data, 0, sizeof(*data));
	data->name = name;
	data->state = PARSER_STATE_IDLE;
	data->current_frame_type = FRAME_TYPE_UNKNOWN;
	protocol->interface = NULL;
	*ready = false;
}

static void bind_dual_online(struct AresInterface *interface, struct AresProtocol *protocol,
			     const char *name, bool *ready)
{
	struct dual_protocol_data *data = protocol->priv_data;

	reset_dual_protocol(protocol, name, ready);
	zassert_ok(ares_bind_interface(interface, protocol), "bind failed");
	*ready = true;
	k_timer_stop(&data->heart_beat_timer);
	data->online = true;
	data->last_receive = k_uptime_get_32();
	data->last_heart_beat = data->last_receive;
}

static void put16(uint8_t *buf, size_t offset, uint16_t value)
{
	GET_16BITS(buf, offset) = value;
}

static void put32(uint8_t *buf, size_t offset, uint32_t value)
{
	GET_32BITS(buf, offset) = value;
}

static int direct_tx_callback_count;
static int direct_tx_callback_status;

static void direct_tx_done(struct AresInterface *interface, struct net_buf *buf, int status,
			   void *user_data)
{
	ARG_UNUSED(interface);
	ARG_UNUSED(buf);

	direct_tx_callback_count++;
	direct_tx_callback_status = status;
	*(int *)user_data = status;
}

ZTEST(ares_communication, test_bind_initializes_interface_before_protocol)
{
	struct AresInterface interface = {
		.name = "observed-interface",
		.api = &observed_interface_api,
	};
	struct AresProtocol protocol = {
		.name = "observed-protocol",
		.api = &observed_protocol_api,
	};

	memset(&bind_obs, 0, sizeof(bind_obs));

	zassert_equal(ares_bind_interface(NULL, &protocol), -EINVAL);
	zassert_equal(ares_bind_interface(&interface, NULL), -EINVAL);
	zassert_ok(ares_bind_interface(&interface, &protocol));

	zassert_equal(interface.protocol, &protocol);
	zassert_equal(protocol.interface, &interface);
	zassert_true(bind_obs.interface_saw_protocol);
	zassert_true(bind_obs.protocol_saw_interface);
	zassert_equal(bind_obs.interface_init_order, 1);
	zassert_equal(bind_obs.protocol_init_order, 2);
}

ZTEST(ares_communication, test_interface_send_with_callback_reports_success_and_failure)
{
	struct fake_transport transport;
	struct AresInterface interface = {
		.name = "direct-interface",
		.api = &fake_interface_api,
		.priv_data = &transport,
	};
	int user_status = 123;
	struct net_buf *buf;

	fake_transport_reset(&transport);
	zassert_ok(interface.api->init(&interface));

	direct_tx_callback_count = 0;
	buf = interface.api->alloc_buf(&interface);
	zassert_not_null(buf);
	net_buf_add_mem(buf, "abc", 3);
	zassert_ok(
		interface.api->send_with_callback(&interface, buf, direct_tx_done, &user_status));
	zassert_equal(transport.send_with_callback_calls, 1);
	zassert_equal(transport.callback_calls, 1);
	zassert_equal(direct_tx_callback_count, 1);
	zassert_equal(user_status, 0);
	zassert_equal(direct_tx_callback_status, 0);
	zassert_equal(transport.last_len, 3);
	zassert_mem_equal(transport.last_frame, "abc", 3);

	transport.fail_next_send = true;
	user_status = 123;
	buf = interface.api->alloc_buf(&interface);
	zassert_not_null(buf);
	net_buf_add_mem(buf, "err", 3);
	zassert_equal(
		interface.api->send_with_callback(&interface, buf, direct_tx_done, &user_status),
		-EIO);
	zassert_equal(transport.callback_calls, 2);
	zassert_equal(user_status, -EIO);
	zassert_equal(direct_tx_callback_status, -EIO);
}

static int sync_status_mask;
static int sync_status_count;

static void sync_status_cb(int status)
{
	sync_status_mask |= status;
	sync_status_count++;
}

ZTEST(ares_communication, test_dual_sync_flush_uses_tx_done_to_release_in_flight)
{
	struct fake_transport transport;
	struct AresInterface interface = {
		.name = "sync-interface",
		.api = &fake_interface_api,
		.priv_data = &transport,
	};
	uint8_t payload[] = {0x10, 0x20, 0x30};
	sync_table_t *pack;

	fake_transport_reset(&transport);
	transport.hold_callback = true;
	bind_dual_online(&interface, &sync_protocol, "sync_protocol", &sync_protocol_ready);

	sync_status_mask = 0;
	sync_status_count = 0;
	pack = dual_sync_add(&sync_protocol, 0x3344, payload, sizeof(payload), sync_status_cb);
	zassert_not_null(pack);
	zassert_equal(sync_status_mask, SYNC_PACK_STATUS_READ);
	zassert_equal(transport.send_with_callback_calls, 1);
	zassert_not_null(transport.held_buf);
	zassert_true(atomic_test_bit(&pack->tx_state, DUAL_TX_IN_FLIGHT));
	zassert_equal(dual_sync_flush(&sync_protocol, pack), -EBUSY);

	fake_complete_held_tx(&interface, 0);
	zassert_false(atomic_test_bit(&pack->tx_state, DUAL_TX_IN_FLIGHT));

	payload[0] = 0x99;
	zassert_ok(dual_sync_flush(&sync_protocol, pack));
	zassert_not_null(transport.held_buf);
	zassert_equal(GET_16BITS(transport.last_frame, SYNC_HEAD_IDX), SYNC_FRAME_HEAD);
	zassert_equal(GET_16BITS(transport.last_frame, SYNC_ID_IDX), 0x3344);
	zassert_equal(transport.last_frame[SYNC_DATA_IDX], 0x99);
	fake_complete_held_tx(&interface, 0);
}

ZTEST(ares_communication, test_dual_sync_requires_callback_capable_interface)
{
	struct fake_transport transport;
	struct AresInterface interface = {
		.name = "no-callback-interface",
		.api = &fake_no_callback_api,
		.priv_data = &transport,
	};
	uint8_t payload[] = {0x01, 0x02};
	sync_table_t *pack;

	fake_transport_reset(&transport);
	bind_dual_online(&interface, &no_callback_protocol, "no_callback_protocol",
			 &no_callback_protocol_ready);

	pack = dual_sync_add(&no_callback_protocol, 0x1122, payload, sizeof(payload), NULL);
	zassert_not_null(pack);
	zassert_equal(dual_sync_flush(&no_callback_protocol, pack), -EBUSY);
	zassert_false(atomic_test_bit(&pack->tx_state, DUAL_TX_IN_FLIGHT));
	zassert_equal(transport.send_calls, 0);
}

ZTEST(ares_communication, test_dual_sync_receive_updates_registered_pack)
{
	struct fake_transport transport;
	struct AresInterface interface = {
		.name = "sync-rx-interface",
		.api = &fake_interface_api,
		.priv_data = &transport,
	};
	uint8_t payload[] = {0xaa, 0xbb, 0xcc};
	uint8_t frame[SYNC_FRAME_LENGTH_OFFSET + sizeof(payload)];
	sync_table_t *pack;

	fake_transport_reset(&transport);
	bind_dual_online(&interface, &sync_protocol, "sync_protocol", &sync_protocol_ready);

	pack = dual_sync_add(&sync_protocol, 0x5566, payload, sizeof(payload), sync_status_cb);
	zassert_not_null(pack);

	sync_status_mask = 0;
	sync_status_count = 0;
	put16(frame, SYNC_HEAD_IDX, SYNC_FRAME_HEAD);
	put16(frame, SYNC_ID_IDX, 0x5566);
	frame[SYNC_DATA_IDX] = 0x01;
	frame[SYNC_DATA_IDX + 1] = 0x02;
	frame[SYNC_DATA_IDX + 2] = 0x03;

	for (size_t i = 0; i < sizeof(frame); i++) {
		sync_protocol.api->handle_byte(&sync_protocol, frame[i]);
	}

	zassert_mem_equal(payload, ((uint8_t[]){0x01, 0x02, 0x03}), sizeof(payload));
	zassert_true((sync_status_mask & SYNC_PACK_STATUS_WRITE) != 0);
	zassert_true((sync_status_mask & SYNC_PACK_STATUS_DONE) != 0);
	zassert_equal(sync_status_count, 2);
}

static uint32_t server_func_arg1;
static uint32_t server_func_arg2;
static uint32_t server_func_arg3;

static uint32_t server_func_cb(uint32_t arg1, uint32_t arg2, uint32_t arg3)
{
	server_func_arg1 = arg1;
	server_func_arg2 = arg2;
	server_func_arg3 = arg3;
	return arg1 + arg2 + arg3;
}

static uint16_t client_ret_id;
static uint16_t client_ret_req_id;
static uint32_t client_ret_value;
static int client_ret_calls;

static void client_ret_cb(uint16_t id, uint16_t req_id, uint32_t ret)
{
	client_ret_id = id;
	client_ret_req_id = req_id;
	client_ret_value = ret;
	client_ret_calls++;
}

ZTEST(ares_communication, test_dual_func_request_reply_and_repl_callback)
{
	struct fake_transport client_transport;
	struct fake_transport server_transport;
	struct AresInterface client_interface = {
		.name = "client-interface",
		.api = &fake_interface_api,
		.priv_data = &client_transport,
	};
	struct AresInterface server_interface = {
		.name = "server-interface",
		.api = &fake_interface_api,
		.priv_data = &server_transport,
	};
	const uint16_t func_id = 0x1234;
	const uint16_t req_id = 0x4a31;
	uint8_t func_frame[FUNC_FRAME_LENGTH];

	fake_transport_reset(&client_transport);
	fake_transport_reset(&server_transport);
	client_transport.peer = &server_interface;
	client_transport.deliver_as_bytes = true;
	server_transport.peer = &client_interface;
	server_transport.deliver_as_bytes = true;
	server_transport.hold_callback = true;

	bind_dual_online(&client_interface, &client_protocol, "client_protocol",
			 &client_protocol_ready);
	bind_dual_online(&server_interface, &server_protocol, "server_protocol",
			 &server_protocol_ready);

	server_func_arg1 = 0;
	server_func_arg2 = 0;
	server_func_arg3 = 0;
	client_ret_calls = 0;
	zassert_ok(dual_ret_cb_set(&client_protocol, client_ret_cb));
	dual_func_add(&server_protocol, func_id, server_func_cb);

	put16(func_frame, FUNC_HEAD_IDX, FUNC_FRAME_HEAD);
	put16(func_frame, FUNC_ID_IDX, func_id);
	put32(func_frame, FUNC_ARG1_IDX, 7);
	put32(func_frame, FUNC_ARG2_IDX, 11);
	put32(func_frame, FUNC_ARG3_IDX, 13);
	put16(func_frame, FUNC_REQ_IDX, req_id);

	for (size_t i = 0; i < sizeof(func_frame); i++) {
		server_protocol.api->handle_byte(&server_protocol, func_frame[i]);
	}

	zassert_equal(server_func_arg1, 7);
	zassert_equal(server_func_arg2, 11);
	zassert_equal(server_func_arg3, 13);
	zassert_equal(server_transport.send_with_callback_calls, 1);
	zassert_not_null(server_transport.held_buf);
	zassert_true(
		atomic_test_bit(&server_protocol_data.func_table[0].tx_state, DUAL_TX_IN_FLIGHT));

	zassert_equal(client_ret_calls, 1);
	zassert_equal(client_ret_id, func_id);
	zassert_equal(client_ret_req_id, req_id);
	zassert_equal(client_ret_value, 31);
	zassert_equal(GET_16BITS(server_transport.last_frame, REPL_REQ_ID_IDX), req_id);

	fake_complete_held_tx(&server_interface, 0);
	zassert_false(
		atomic_test_bit(&server_protocol_data.func_table[0].tx_state, DUAL_TX_IN_FLIGHT));
}

ZTEST(ares_communication, test_dual_func_call_serializes_and_delivers_request)
{
	struct fake_transport client_transport;
	struct fake_transport server_transport;
	struct AresInterface client_interface = {
		.name = "client-interface",
		.api = &fake_interface_api,
		.priv_data = &client_transport,
	};
	struct AresInterface server_interface = {
		.name = "server-interface",
		.api = &fake_interface_api,
		.priv_data = &server_transport,
	};
	int req_id;

	fake_transport_reset(&client_transport);
	fake_transport_reset(&server_transport);
	client_transport.peer = &server_interface;
	server_transport.peer = &client_interface;

	bind_dual_online(&client_interface, &client_protocol, "client_protocol",
			 &client_protocol_ready);
	bind_dual_online(&server_interface, &server_protocol, "server_protocol",
			 &server_protocol_ready);

	dual_func_add(&server_protocol, 0x2233, server_func_cb);
	req_id = dual_func_call(&client_protocol, 0x2233, 3, 4, 5);

	zassert_true(req_id >= 0);
	zassert_equal(client_transport.send_calls, 1);
	zassert_equal(server_func_arg1, 3);
	zassert_equal(server_func_arg2, 4);
	zassert_equal(server_func_arg3, 5);
	zassert_equal(server_transport.send_with_callback_calls, 1);
	zassert_equal(GET_16BITS(client_transport.last_frame, FUNC_HEAD_IDX), FUNC_FRAME_HEAD);
	zassert_equal(GET_16BITS(client_transport.last_frame, FUNC_ID_IDX), 0x2233);
}

ZTEST(ares_communication, test_disconnect_event_clears_sync_tx_in_flight)
{
	struct fake_transport transport;
	struct AresInterface interface = {
		.name = "sync-disconnect-interface",
		.api = &fake_interface_api,
		.priv_data = &transport,
	};
	uint8_t payload[] = {0xde, 0xad};
	sync_table_t *pack;

	fake_transport_reset(&transport);
	transport.hold_callback = true;
	bind_dual_online(&interface, &sync_protocol, "sync_protocol", &sync_protocol_ready);

	pack = dual_sync_add(&sync_protocol, 0x8899, payload, sizeof(payload), NULL);
	zassert_not_null(pack);
	zassert_true(atomic_test_bit(&pack->tx_state, DUAL_TX_IN_FLIGHT));

	sync_protocol.api->event(&sync_protocol, ARES_PROTOCOL_EVENT_DISCONNECTED);
	zassert_false(sync_protocol_data.online);
	zassert_false(atomic_test_bit(&pack->tx_state, DUAL_TX_IN_FLIGHT));

	fake_complete_held_tx(&interface, 0);
}

ZTEST(ares_communication, test_disconnect_event_clears_func_tx_in_flight)
{
	struct fake_transport transport;
	struct AresInterface interface = {
		.name = "disconnect-interface",
		.api = &fake_interface_api,
		.priv_data = &transport,
	};
	const uint16_t func_id = 0x7777;
	uint8_t func_frame[FUNC_FRAME_LENGTH];

	fake_transport_reset(&transport);
	transport.hold_callback = true;
	bind_dual_online(&interface, &disconnect_protocol, "disconnect_protocol",
			 &disconnect_protocol_ready);

	dual_func_add(&disconnect_protocol, func_id, server_func_cb);
	put16(func_frame, FUNC_HEAD_IDX, FUNC_FRAME_HEAD);
	put16(func_frame, FUNC_ID_IDX, func_id);
	put32(func_frame, FUNC_ARG1_IDX, 1);
	put32(func_frame, FUNC_ARG2_IDX, 2);
	put32(func_frame, FUNC_ARG3_IDX, 3);
	put16(func_frame, FUNC_REQ_IDX, 0x0102);

	for (size_t i = 0; i < sizeof(func_frame); i++) {
		disconnect_protocol.api->handle_byte(&disconnect_protocol, func_frame[i]);
	}

	zassert_true(atomic_test_bit(&disconnect_protocol_data.func_table[0].tx_state,
				     DUAL_TX_IN_FLIGHT));
	disconnect_protocol.api->event(&disconnect_protocol, ARES_PROTOCOL_EVENT_DISCONNECTED);
	zassert_false(disconnect_protocol_data.online);
	zassert_false(atomic_test_bit(&disconnect_protocol_data.func_table[0].tx_state,
				      DUAL_TX_IN_FLIGHT));

	fake_complete_held_tx(&interface, 0);
}

ZTEST_SUITE(ares_communication, NULL, NULL, NULL, NULL, NULL);
