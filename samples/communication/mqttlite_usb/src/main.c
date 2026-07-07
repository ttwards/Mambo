/*
 * Copyright (c) 2026 ttwards
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <ares/ares_comm.h>
#include <ares/interface/usb/usb_bulk.h>
#include <ares/protocol/mqttlite/mqttlite_protocol.h>

LOG_MODULE_REGISTER(mqttlite_usb_sample, LOG_LEVEL_INF);

#define TOPIC_HEARTBEAT    1
#define TOPIC_HOST_COMMAND 2

ARES_BULK_INTERFACE_DEFINE(usb_bulk_interface);
ARES_MQTTLITE_PROTOCOL_DEFINE(mqttlite_protocol);

static void command_cb(struct AresProtocol *protocol, const char *topic, const uint8_t *payload,
		       uint16_t payload_len, enum ares_mqttlite_qos qos, void *user_data)
{
	char text[65];
	size_t len = MIN(payload_len, sizeof(text) - 1);

	ARG_UNUSED(protocol);
	ARG_UNUSED(user_data);

	memcpy(text, payload, len);
	text[len] = '\0';

	LOG_INF("RX topic=%s qos=%u payload=%s", topic, qos, text);
}

int main(void)
{
	uint32_t counter = 0;
	int ret;

	LOG_INF("ARES MQTT-like USB Bulk sample starting");

	ret = ares_bind_interface(&usb_bulk_interface, &mqttlite_protocol);
	if (ret != 0) {
		LOG_ERR("Failed to bind USB bulk interface (%d)", ret);
		return ret;
	}

	ret = ares_mqttlite_register_topic(&mqttlite_protocol, TOPIC_HEARTBEAT,
					   "robot/mqttlite/heartbeat");
	if (ret != 0) {
		LOG_ERR("Failed to register heartbeat topic (%d)", ret);
		return ret;
	}

	ret = ares_mqttlite_register_topic(&mqttlite_protocol, TOPIC_HOST_COMMAND, "host/command");
	if (ret != 0) {
		LOG_ERR("Failed to register command topic (%d)", ret);
		return ret;
	}

	ret = ares_mqttlite_subscribe_id(&mqttlite_protocol, TOPIC_HOST_COMMAND, command_cb, NULL);
	if (ret != 0) {
		LOG_ERR("Failed to subscribe host/command (%d)", ret);
		return ret;
	}

	while (1) {
		char payload[32];
		struct ares_mqttlite_publish_buffer pub;
		size_t payload_len;

		snprintk(payload, sizeof(payload), "heartbeat:%u", counter++);
		payload_len = strlen(payload);

		ret = ares_mqttlite_publish_prepare_id(&mqttlite_protocol, TOPIC_HEARTBEAT,
						       payload_len, &pub);
		if (ret == 0) {
			memcpy(pub.payload, payload, payload_len);
			ret = ares_mqttlite_publish_commit(&mqttlite_protocol, &pub);
		}
		if (ret < 0) {
			LOG_WRN("Publish failed (%d)", ret);
		} else {
			LOG_INF("QoS0 heartbeat published");
		}

		k_sleep(K_SECONDS(1));
	}

	return 0;
}
