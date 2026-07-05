/*
 * Copyright (c) 2026 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#define DM_NODE   DT_NODELABEL(dm0)
#define MI_NODE   DT_NODELABEL(mi0)
#define RS_NODE   DT_NODELABEL(rs0)
#define LK_NODE   DT_NODELABEL(lk0)
#define DJI_NODE  DT_NODELABEL(dji0)
#define DJI2_NODE DT_NODELABEL(dji1)
#define DJI3_NODE DT_NODELABEL(dji2)

#define DM_DEV   DEVICE_DT_GET(DM_NODE)
#define MI_DEV   DEVICE_DT_GET(MI_NODE)
#define RS_DEV   DEVICE_DT_GET(RS_NODE)
#define LK_DEV   DEVICE_DT_GET(LK_NODE)
#define DJI_DEV  DEVICE_DT_GET(DJI_NODE)
#define DJI2_DEV DEVICE_DT_GET(DJI2_NODE)
#define DJI3_DEV DEVICE_DT_GET(DJI3_NODE)

#define DM_TX_ID   DT_PROP(DM_NODE, tx_id)
#define DM_RX_ID   DT_PROP(DM_NODE, rx_id)
#define MI_ID      DT_PROP(MI_NODE, id)
#define RS_TX_ID   DT_PROP(RS_NODE, tx_id)
#define LK_ID      DT_PROP(LK_NODE, id)
#define DJI_TX_ID  DT_PROP(DJI_NODE, tx_id)
#define DJI_RX_ID  DT_PROP(DJI_NODE, rx_id)
#define DJI2_TX_ID DT_PROP(DJI2_NODE, tx_id)
#define DJI2_RX_ID DT_PROP(DJI2_NODE, rx_id)
#define DJI3_TX_ID DT_PROP(DJI3_NODE, tx_id)
#define DJI3_RX_ID DT_PROP(DJI3_NODE, rx_id)

#define MI_MODE_FEEDBACK     0x02U
#define RS_MODE_FEEDBACK     0x02U
#define RS_MODE_MOTOR_ENABLE 0x03U
#define RS_MODE_MOTOR_REPORT 0x18U
#define RS_STATE_RESET       0x00U
#define RS_STATE_MOTOR       0x02U

#define SIM_FILTER_MAX 16
#define SIM_TX_MAX     128
#define SIM_CAN_DLEN   8U

#define DM_RATE_WINDOW_MS  300
#define MI_RATE_WINDOW_MS  300
#define RS_RATE_WINDOW_MS  300
#define DJI_RATE_WINDOW_MS 300

#define CONTROL_LATENCY_MS         1000
#define DJI_CONTROL_LATENCY_MS     30
#define ONLINE_RECOVERY_MS         30
#define REPLY_RESPONDER_STACK_SIZE 1024

struct sim_filter {
	bool used;
	can_rx_callback_t cb;
	void *user_data;
	struct can_filter filter;
};

struct sim_tx_record {
	struct can_frame frame;
	uint32_t uptime_ms;
	uint32_t cycle;
};

static struct sim_filter sim_filters[SIM_FILTER_MAX];
static struct sim_tx_record sim_tx_history[SIM_TX_MAX];
static uint32_t sim_tx_count;
static struct k_spinlock sim_lock;

struct test_can_config {
	struct can_driver_config common;
};

struct test_can_data {
	struct can_driver_data common;
	bool started;
};

struct motor_controller {
	struct device *can_dev;
	int rx_ids[8];
	bool full[8];
	int8_t mapping[8][4];
	uint8_t flags;
	uint8_t mask[8];
	struct device *motor_devs[8];
	struct k_work full_handle;
};

extern struct motor_controller ctrl_structs[];
void dji_tx_handler(struct k_work *work);
void lk_tx_data_handler(struct k_work *work);

static bool frame_matches_filter(const struct can_frame *frame, const struct can_filter *filter)
{
	bool frame_ide = (frame->flags & CAN_FRAME_IDE) != 0U;
	bool filter_ide = (filter->flags & CAN_FILTER_IDE) != 0U;

	if (frame_ide != filter_ide) {
		return false;
	}

	return (frame->id & filter->mask) == (filter->id & filter->mask);
}

static int sim_can_add_rx_filter(const struct device *dev, can_rx_callback_t cb, void *user_data,
				 const struct can_filter *filter)
{
	ARG_UNUSED(dev);

	k_spinlock_key_t key = k_spin_lock(&sim_lock);

	for (int i = 0; i < ARRAY_SIZE(sim_filters); i++) {
		if (!sim_filters[i].used) {
			sim_filters[i] = (struct sim_filter){
				.used = true,
				.cb = cb,
				.user_data = user_data,
				.filter = *filter,
			};
			k_spin_unlock(&sim_lock, key);
			return i;
		}
	}

	k_spin_unlock(&sim_lock, key);
	return -ENOMEM;
}

static int sim_can_send(const struct device *dev, const struct can_frame *frame,
			k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	ARG_UNUSED(timeout);

	k_spinlock_key_t key = k_spin_lock(&sim_lock);
	sim_tx_history[sim_tx_count % ARRAY_SIZE(sim_tx_history)] = (struct sim_tx_record){
		.frame = *frame,
		.uptime_ms = (uint32_t)k_uptime_get(),
		.cycle = k_cycle_get_32(),
	};
	sim_tx_count++;
	k_spin_unlock(&sim_lock, key);

	if (callback != NULL) {
		callback(dev, 0, user_data);
	}

	return 0;
}

static int test_can_start(const struct device *dev)
{
	struct test_can_data *data = dev->data;

	data->started = true;
	return 0;
}

static int test_can_stop(const struct device *dev)
{
	struct test_can_data *data = dev->data;

	data->started = false;
	return 0;
}

static int test_can_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL;
	return 0;
}

static int test_can_set_mode(const struct device *dev, can_mode_t mode)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(mode);

	return 0;
}

static int test_can_set_timing(const struct device *dev, const struct can_timing *timing)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(timing);

	return 0;
}

static int test_can_get_state(const struct device *dev, enum can_state *state,
			      struct can_bus_err_cnt *err_cnt)
{
	struct test_can_data *data = dev->data;

	if (state != NULL) {
		*state = data->started ? CAN_STATE_ERROR_ACTIVE : CAN_STATE_STOPPED;
	}
	if (err_cnt != NULL) {
		err_cnt->tx_err_cnt = 0U;
		err_cnt->rx_err_cnt = 0U;
	}
	return 0;
}

static int test_can_get_core_clock(const struct device *dev, uint32_t *rate)
{
	ARG_UNUSED(dev);

	*rate = MHZ(80);
	return 0;
}

static int test_can_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(ide);

	return SIM_FILTER_MAX;
}

static int test_can_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static DEVICE_API(can, test_can_api) = {
	.start = test_can_start,
	.stop = test_can_stop,
	.get_capabilities = test_can_get_capabilities,
	.set_mode = test_can_set_mode,
	.set_timing = test_can_set_timing,
	.send = sim_can_send,
	.add_rx_filter = sim_can_add_rx_filter,
	.get_state = test_can_get_state,
	.get_core_clock = test_can_get_core_clock,
	.get_max_filters = test_can_get_max_filters,
};

#define DT_DRV_COMPAT ares_test_can

#define TEST_CAN_INIT(inst)                                                                        \
	static const struct test_can_config test_can_config_##inst = {                             \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000),                         \
	};                                                                                         \
	static struct test_can_data test_can_data_##inst;                                          \
	CAN_DEVICE_DT_INST_DEFINE(inst, test_can_init, NULL, &test_can_data_##inst,                \
				  &test_can_config_##inst, POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,  \
				  &test_can_api);

DT_INST_FOREACH_STATUS_OKAY(TEST_CAN_INIT)

static void sim_reset_tx_history(void)
{
	k_spinlock_key_t key = k_spin_lock(&sim_lock);

	memset(sim_tx_history, 0, sizeof(sim_tx_history));
	sim_tx_count = 0;

	k_spin_unlock(&sim_lock, key);
}

static uint32_t sim_current_tx_count(void)
{
	k_spinlock_key_t key = k_spin_lock(&sim_lock);
	uint32_t count = sim_tx_count;

	k_spin_unlock(&sim_lock, key);
	return count;
}

static bool sim_tx_seen_since(uint32_t start, bool (*match)(const struct can_frame *frame))
{
	k_spinlock_key_t key = k_spin_lock(&sim_lock);
	uint32_t end = sim_tx_count;
	bool seen = false;

	for (uint32_t i = start; i < end; i++) {
		const struct can_frame *frame =
			&sim_tx_history[i % ARRAY_SIZE(sim_tx_history)].frame;

		if (match(frame)) {
			seen = true;
			break;
		}
	}

	k_spin_unlock(&sim_lock, key);
	return seen;
}

static bool sim_first_tx_since(uint32_t start, bool (*match)(const struct can_frame *frame),
			       struct sim_tx_record *record, uint32_t *index)
{
	k_spinlock_key_t key = k_spin_lock(&sim_lock);
	uint32_t end = sim_tx_count;
	bool seen = false;

	for (uint32_t i = start; i < end; i++) {
		const struct sim_tx_record *candidate =
			&sim_tx_history[i % ARRAY_SIZE(sim_tx_history)];

		if (match(&candidate->frame)) {
			if (record != NULL) {
				*record = *candidate;
			}
			if (index != NULL) {
				*index = i;
			}
			seen = true;
			break;
		}
	}

	k_spin_unlock(&sim_lock, key);
	return seen;
}

static uint32_t sim_count_tx_since(uint32_t start, bool (*match)(const struct can_frame *frame))
{
	k_spinlock_key_t key = k_spin_lock(&sim_lock);
	uint32_t end = sim_tx_count;
	uint32_t count = 0;

	for (uint32_t i = start; i < end; i++) {
		const struct can_frame *frame =
			&sim_tx_history[i % ARRAY_SIZE(sim_tx_history)].frame;

		if (match(frame)) {
			count++;
		}
	}

	k_spin_unlock(&sim_lock, key);
	return count;
}

static void sim_dump_tx_since(uint32_t start)
{
	k_spinlock_key_t key = k_spin_lock(&sim_lock);
	uint32_t end = sim_tx_count;

	for (uint32_t i = start; i < end; i++) {
		const struct sim_tx_record *record =
			&sim_tx_history[i % ARRAY_SIZE(sim_tx_history)];
		const struct can_frame *frame = &record->frame;

		TC_PRINT("tx[%u] t=%u id=0x%x flags=0x%x dlc=%u data=%02x %02x %02x %02x %02x %02x "
			 "%02x %02x\n",
			 i, record->uptime_ms, frame->id, frame->flags, frame->dlc, frame->data[0],
			 frame->data[1], frame->data[2], frame->data[3], frame->data[4],
			 frame->data[5], frame->data[6], frame->data[7]);
	}

	k_spin_unlock(&sim_lock, key);
}

struct expected_tx {
	bool (*base_match)(const struct can_frame *frame);
	uint32_t id;
	uint32_t id_mask;
	uint8_t data[CAN_MAX_DLEN];
	uint8_t dlc;
};

static struct expected_tx expected_tx;

static bool match_expected_tx(const struct can_frame *frame)
{
	if (!expected_tx.base_match(frame)) {
		return false;
	}
	if (expected_tx.id_mask != 0U &&
	    ((frame->id & expected_tx.id_mask) != (expected_tx.id & expected_tx.id_mask))) {
		return false;
	}

	return frame->dlc == expected_tx.dlc &&
	       memcmp(frame->data, expected_tx.data, expected_tx.dlc) == 0;
}

static void wait_for_tx_after(uint32_t start, bool (*match)(const struct can_frame *frame),
			      const char *name)
{
	int64_t deadline = k_uptime_get() + 500;

	while (k_uptime_get() < deadline) {
		if (sim_tx_seen_since(start, match)) {
			return;
		}
		k_sleep(K_MSEC(1));
	}
	if (sim_tx_seen_since(start, match)) {
		return;
	}

	sim_dump_tx_since(start);
	zassert_true(false, "%s did not transmit the expected CAN frame (tx count %u -> %u)", name,
		     start, sim_current_tx_count());
}

static void expect_payload_sequence_step(const char *name,
					 bool (*base_match)(const struct can_frame *frame),
					 uint32_t expected_id, uint32_t expected_id_mask,
					 const uint8_t expected[CAN_MAX_DLEN], uint32_t set_at_ms,
					 void (*emit_feedback)(void), int latency_limit_ms,
					 uint32_t *start, uint32_t *previous_match)
{
	struct sim_tx_record record;
	uint32_t index = 0;
	uint32_t stale_count = 0;
	int64_t deadline = k_uptime_get() + latency_limit_ms;

	expected_tx.base_match = base_match;
	expected_tx.id = expected_id;
	expected_tx.id_mask = expected_id_mask;
	expected_tx.dlc = SIM_CAN_DLEN;
	memcpy(expected_tx.data, expected, SIM_CAN_DLEN);

	while (k_uptime_get() < deadline) {
		if (sim_first_tx_since(*start, match_expected_tx, &record, &index)) {
			goto matched;
		}
		if (emit_feedback != NULL &&
		    sim_first_tx_since(*start, base_match, &record, &index)) {
			zassert_true(stale_count < 300U,
				     "%s sent too many stale control frames before update", name);
			stale_count++;
			emit_feedback();
			*start = index + 1U;
			continue;
		}
		k_sleep(K_MSEC(1));
	}
	if (sim_first_tx_since(*start, match_expected_tx, &record, &index)) {
		goto matched;
	}

	sim_dump_tx_since(*start);
	TC_PRINT("%s expected id=0x%x mask=0x%x dlc=%u data=%02x %02x %02x %02x %02x %02x %02x "
		 "%02x\n",
		 name, expected_tx.id, expected_tx.id_mask, expected_tx.dlc, expected_tx.data[0],
		 expected_tx.data[1], expected_tx.data[2], expected_tx.data[3], expected_tx.data[4],
		 expected_tx.data[5], expected_tx.data[6], expected_tx.data[7]);
	zassert_true(false, "%s did not transmit the expected CAN frame (tx count %u -> %u)", name,
		     *start, sim_current_tx_count());
	return;

matched:
	if (*previous_match != UINT32_MAX) {
		zassert_true(index > *previous_match, "%s matched frame order regressed", name);
	}
	zassert_true(record.uptime_ms - set_at_ms <= latency_limit_ms,
		     "%s control frame latency exceeded %d ms", name, latency_limit_ms);

	*previous_match = index;
	*start = index + 1U;

	if (emit_feedback != NULL) {
		emit_feedback();
	}
}

static void sim_emit_frame(const struct device *can_dev, const struct can_frame *frame)
{
	struct sim_filter filters[SIM_FILTER_MAX];

	k_spinlock_key_t key = k_spin_lock(&sim_lock);
	memcpy(filters, sim_filters, sizeof(filters));
	k_spin_unlock(&sim_lock, key);

	for (int i = 0; i < ARRAY_SIZE(filters); i++) {
		if (filters[i].used && frame_matches_filter(frame, &filters[i].filter)) {
			struct can_frame copy = *frame;

			filters[i].cb(can_dev, &copy, filters[i].user_data);
		}
	}
}

static void read_motor_status(const struct device *dev, motor_status_t *status)
{
	const struct motor_driver_api *api = dev->api;
	int ret = api->motor_get(dev, status);

	zassert_true(ret == 0 || ret == -ENODEV, "motor_get returned unexpected error %d", ret);
}

static bool motor_online(const struct device *dev)
{
	motor_status_t status = {0};

	read_motor_status(dev, &status);
	return status.online;
}

static bool motor_status_enabled(const struct device *dev)
{
	motor_status_t status = {0};

	read_motor_status(dev, &status);
	return status.enabled;
}

static bool motor_requested_enabled(const struct device *dev)
{
	const struct motor_driver_data *common = dev->data;

	return common->link.requested_enabled;
}

static void driver_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	const struct motor_driver_api *api = dev->api;

	api->motor_control(dev, cmd);
}

static void expect_online(const struct device *dev, bool expected, const char *name)
{
	bool online = motor_online(dev);

	zassert_equal(online, expected, "%s online state mismatch", name);
}

static void expect_requested_enabled(const struct device *dev, bool expected, const char *name)
{
	bool requested_enabled = motor_requested_enabled(dev);

	zassert_equal(requested_enabled, expected, "%s requested_enabled state mismatch", name);
}

static void expect_status_enabled(const struct device *dev, bool expected, const char *name)
{
	bool enabled = motor_status_enabled(dev);

	zassert_equal(enabled, expected, "%s status.enabled state mismatch", name);
}

static void force_motor_offline(const struct device *dev)
{
	struct motor_driver_data *common = dev->data;

	common->link.online = false;
	common->link.missed = 0;
}

static void wait_for_online_state(const struct device *dev, bool expected, int timeout_ms,
				  const char *name)
{
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline) {
		if (motor_online(dev) == expected) {
			return;
		}
		k_sleep(K_MSEC(1));
	}

	zassert_true(false, "%s did not reach expected online state %d", name, expected);
}

static bool match_dm_tx(const struct can_frame *frame)
{
	return ((frame->flags & CAN_FRAME_IDE) == 0U) && frame->id == DM_TX_ID;
}

static bool match_mi_tx(const struct can_frame *frame)
{
	return ((frame->flags & CAN_FRAME_IDE) != 0U) && ((frame->id & 0xFFU) == MI_ID);
}

static bool match_rs_tx(const struct can_frame *frame)
{
	return ((frame->flags & CAN_FRAME_IDE) != 0U) && ((frame->id & 0xFFU) == RS_TX_ID);
}

static bool match_rs_msg_type(const struct can_frame *frame, uint8_t msg_type)
{
	return match_rs_tx(frame) && (((frame->id >> 24) & 0x1FU) == msg_type);
}

static bool match_rs_enable_tx(const struct can_frame *frame)
{
	return match_rs_msg_type(frame, RS_MODE_MOTOR_ENABLE);
}

static bool match_rs_auto_report_tx(const struct can_frame *frame)
{
	return match_rs_msg_type(frame, RS_MODE_MOTOR_REPORT);
}

static bool match_lk_tx(const struct can_frame *frame)
{
	return ((frame->flags & CAN_FRAME_IDE) == 0U) && frame->id == (0x140U + LK_ID);
}

static bool match_dji_tx(const struct can_frame *frame)
{
	return ((frame->flags & CAN_FRAME_IDE) == 0U) && frame->id == DJI_TX_ID;
}

static bool match_dji3_tx(const struct can_frame *frame)
{
	return ((frame->flags & CAN_FRAME_IDE) == 0U) && frame->id == DJI3_TX_ID;
}

static int test_float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;

	if (x > x_max) {
		x = x_max;
	} else if (x < x_min) {
		x = x_min;
	}

	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static int test_float_to_int(float x, float x_min, float x_max, int bits)
{
	int target_min = -(1 << (bits - 1));
	int target_max = (1 << (bits - 1)) - 1;
	float target_span = (float)(target_max - target_min);

	if (x > x_max) {
		x = x_max;
	} else if (x < x_min) {
		x = x_min;
	}

	return (int)((x - x_min) / (x_max - x_min) * target_span + (float)target_min);
}

static void expected_dm_mit(float rpm, uint8_t data[CAN_MAX_DLEN])
{
	uint16_t pos_tmp = test_float_to_uint(0.0f, -12.5f, 12.5f, 16);
	uint16_t vel_tmp = test_float_to_uint(RPM2RADPS(rpm), -12.5f, 12.5f, 12);
	uint16_t kp_tmp = test_float_to_uint(0.0f, 0.0f, 500.0f, 12);
	uint16_t kd_tmp = test_float_to_uint(0.0f, 0.0f, 5.0f, 12);
	uint16_t tor_tmp = test_float_to_uint(0.0f, -12.0f, 12.0f, 12);

	data[0] = pos_tmp >> 8;
	data[1] = pos_tmp;
	data[2] = vel_tmp >> 4;
	data[3] = ((vel_tmp & 0x0FU) << 4) | (kp_tmp >> 8);
	data[4] = kp_tmp;
	data[5] = kd_tmp >> 4;
	data[6] = ((kd_tmp & 0x0FU) << 4) | (tor_tmp >> 8);
	data[7] = tor_tmp;
}

static void expected_mi_mit(float rpm, uint8_t data[CAN_MAX_DLEN])
{
	uint16_t pos_tmp = test_float_to_uint(0.0f, -12.5f, 12.5f, 16);
	uint16_t vel_tmp = test_float_to_uint(RPM2RADPS(rpm), -30.0f, 30.0f, 16);
	uint16_t kp_tmp = test_float_to_uint(0.0f, 0.0f, 500.0f, 16);
	uint16_t kd_tmp = test_float_to_uint(0.0f, 0.0f, 5.0f, 16);

	data[0] = (pos_tmp >> 8) & 0xFF;
	data[1] = pos_tmp & 0xFF;
	data[2] = (vel_tmp >> 8) & 0xFF;
	data[3] = vel_tmp & 0xFF;
	data[4] = (kp_tmp >> 8) & 0xFF;
	data[5] = kp_tmp & 0xFF;
	data[6] = (kd_tmp >> 8) & 0xFF;
	data[7] = kd_tmp & 0xFF;
}

static uint32_t expected_mi_mit_id(void)
{
	uint16_t tor_tmp = test_float_to_uint(0.0f, -12.0f, 12.0f, 16);

	return MI_ID | ((uint32_t)tor_tmp << 8) | (0x01U << 24);
}

static void expected_rs_mit(float rpm, uint8_t data[CAN_MAX_DLEN])
{
	uint16_t pos_tmp = test_float_to_uint(0.0f, -12.57f, 12.57f, 16);
	uint16_t vel_tmp = test_float_to_uint(RPM2RADPS(rpm), -44.0f, 44.0f, 16);
	uint16_t kp_tmp = test_float_to_uint(0.0f, 0.0f, 500.0f, 16);
	uint16_t kd_tmp = test_float_to_uint(0.0f, 0.0f, 5.0f, 16);

	data[0] = (pos_tmp >> 8) & 0xFF;
	data[1] = pos_tmp & 0xFF;
	data[2] = (vel_tmp >> 8) & 0xFF;
	data[3] = vel_tmp & 0xFF;
	data[4] = (kp_tmp >> 8) & 0xFF;
	data[5] = kp_tmp & 0xFF;
	data[6] = (kd_tmp >> 8) & 0xFF;
	data[7] = kd_tmp & 0xFF;
}

static uint32_t expected_rs_mit_id(void)
{
	uint16_t tor_tmp = test_float_to_uint(0.0f, -17.0f, 17.0f, 16);

	return RS_TX_ID | (((uint32_t)tor_tmp & 0xFFU) << 8) |
	       ((((uint32_t)tor_tmp >> 8) & 0xFFU) << 16) | (0x01U << 24);
}

static void expected_lk_speed(float rpm, uint8_t data[CAN_MAX_DLEN])
{
	int16_t torque = (int16_t)test_float_to_int(3.0f, 0.0f, 12.0f, 12);
	int32_t speed = (int32_t)(rpm * 6.0f * 100.0f);

	data[0] = 0xA2;
	data[1] = 0x00;
	data[2] = torque & 0xFF;
	data[3] = (torque >> 8) & 0xFF;
	data[4] = speed & 0xFF;
	data[5] = (speed >> 8) & 0xFF;
	data[6] = (speed >> 16) & 0xFF;
	data[7] = (speed >> 24) & 0xFF;
}

static int16_t dji_expected_current(float target_rpm, float report_rpm)
{
	float torque = (target_rpm - report_rpm) * 0.0001f;
	float current = torque * 45000.0422411f;

	if (current > INT16_MAX) {
		return INT16_MAX;
	}
	if (current < INT16_MIN) {
		return INT16_MIN;
	}
	return (int16_t)current;
}

static void expected_dji_current(int16_t current, uint8_t data[CAN_MAX_DLEN])
{
	memset(data, 0, CAN_MAX_DLEN);
	data[0] = (current >> 8) & 0xFF;
	data[1] = current & 0xFF;
}

static void expected_dji_current_slot(int16_t current, uint8_t slot, uint8_t data[CAN_MAX_DLEN])
{
	data[slot * 2U] = (current >> 8) & 0xFF;
	data[slot * 2U + 1U] = current & 0xFF;
}

static void configure_dji_speed_test_limits(const struct device *dev)
{
	int ret = motor_set(dev, &(motor_setpoint_t){
					 .target = MOTOR_TARGET_NONE,
					 .speed_limit = {-3000.0f, 3000.0f},
					 .torque_limit = {-1.0f, 1.0f},
				 });

	zassert_equal(ret, 0, "DJI rejected test limits %d", ret);
}

static void emit_dm_feedback_enabled(bool enabled)
{
	struct can_frame frame = {
		.id = DM_RX_ID & CAN_STD_ID_MASK,
		.dlc = 8,
		.data = {(enabled ? 0x10U : 0x00U) | (DM_TX_ID & 0x0FU), 0, 0, 0, 0, 0, 0, 0},
	};

	sim_emit_frame(DEVICE_DT_GET(DT_NODELABEL(fake_can)), &frame);
}

static void emit_dm_feedback(void)
{
	emit_dm_feedback_enabled(true);
}

static void emit_mi_feedback(void)
{
	struct can_frame frame = {
		.id = (MI_MODE_FEEDBACK << 24) | (MI_ID << 8),
		.flags = CAN_FRAME_IDE,
		.dlc = 8,
	};

	sim_emit_frame(DEVICE_DT_GET(DT_NODELABEL(fake_can)), &frame);
}

static void emit_rs_feedback_state(uint8_t mode_state)
{
	struct can_frame frame = {
		.id = (RS_MODE_FEEDBACK << 24) | ((mode_state & 0x03U) << 22) |
		      (RS_TX_ID << 8),
		.flags = CAN_FRAME_IDE,
		.dlc = 8,
	};

	sim_emit_frame(DEVICE_DT_GET(DT_NODELABEL(fake_can)), &frame);
}

static void emit_rs_feedback(void)
{
	emit_rs_feedback_state(RS_STATE_MOTOR);
}

static void emit_rs_reset_feedback(void)
{
	emit_rs_feedback_state(RS_STATE_RESET);
}

static void emit_lk_feedback(void)
{
	struct can_frame frame = {
		.id = 0x140U + LK_ID,
		.dlc = 8,
		.data = {0x9C, 25, 0, 0, 0, 0, 0, 0},
	};

	sim_emit_frame(DEVICE_DT_GET(DT_NODELABEL(fake_can)), &frame);
}

static void emit_dji_report_for(uint32_t rx_id, int16_t rpm)
{
	struct can_frame frame = {
		.id = rx_id,
		.dlc = 8,
		.data = {0x20, 0, (rpm >> 8) & 0xFF, rpm & 0xFF, 0, 0, 25, 0},
	};

	sim_emit_frame(DEVICE_DT_GET(DT_NODELABEL(fake_can)), &frame);
}

static void emit_dji_report_with_rpm(int16_t rpm)
{
	emit_dji_report_for(DJI_RX_ID, rpm);
}

static void emit_dji_report(void)
{
	emit_dji_report_with_rpm(0);
}

static void service_dji_tx_work(void)
{
	dji_tx_handler(&ctrl_structs[0].full_handle);
}

static uint32_t wait_for_online_state_timed(const struct device *dev, bool expected, int timeout_ms,
					    const char *name)
{
	uint32_t start = (uint32_t)k_uptime_get();
	int64_t deadline = k_uptime_get() + timeout_ms;

	while (k_uptime_get() < deadline) {
		if (motor_online(dev) == expected) {
			return (uint32_t)k_uptime_get() - start;
		}
		k_sleep(K_MSEC(1));
	}

	zassert_true(false, "%s did not reach expected online state %d", name, expected);
	return UINT32_MAX;
}

static void verify_request_reply_motor(const struct device *dev, const char *name,
				       bool (*match_tx)(const struct can_frame *frame),
				       void (*emit_feedback)(void), int offline_timeout_ms)
{
	uint32_t tx_start;
	uint32_t recovery_ms;
	bool was_online;

	zassert_true(device_is_ready(dev), "%s device is not ready", name);
	zassert_not_null(dev->api, "%s device has no driver API", name);
	driver_motor_control(dev, DISABLE_MOTOR);
	k_sleep(K_MSEC(15));
	was_online = motor_online(dev);

	tx_start = sim_current_tx_count();
	driver_motor_control(dev, ENABLE_MOTOR);
	wait_for_tx_after(tx_start, match_tx, name);
	if (!was_online) {
		expect_online(dev, false, name);
	}

	emit_feedback();
	k_sleep(K_MSEC(10));
	expect_online(dev, true, name);

	if (offline_timeout_ms <= 0) {
		driver_motor_control(dev, DISABLE_MOTOR);
		k_sleep(K_MSEC(10));
		return;
	}

	wait_for_online_state(dev, false, offline_timeout_ms, name);

	tx_start = sim_current_tx_count();
	wait_for_tx_after(tx_start, match_tx, name);
	emit_feedback();
	recovery_ms = wait_for_online_state_timed(dev, true, ONLINE_RECOVERY_MS, name);
	zassert_true(recovery_ms <= ONLINE_RECOVERY_MS, "%s recovery took %u ms", name,
		     recovery_ms);

	driver_motor_control(dev, DISABLE_MOTOR);
	k_sleep(K_MSEC(10));
}

static bool keep_reply_motor_unblocked(bool (*match_tx)(const struct can_frame *frame),
				       void (*emit_feedback)(void), uint32_t *cursor)
{
	struct sim_tx_record record;
	uint32_t index;
	bool replied = false;

	while (sim_first_tx_since(*cursor, match_tx, &record, &index)) {
		ARG_UNUSED(record);
		*cursor = index + 1U;
		emit_feedback();
		replied = true;
	}

	return replied;
}

struct reply_responder {
	bool (*match_tx)(const struct can_frame *frame);
	void (*emit_feedback)(void);
	uint32_t cursor;
	volatile bool stop;
};

static K_THREAD_STACK_DEFINE(reply_responder_stack, REPLY_RESPONDER_STACK_SIZE);
static struct k_thread reply_responder_thread;

static void reply_responder_entry(void *p1, void *p2, void *p3)
{
	struct reply_responder *responder = p1;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (!responder->stop) {
		keep_reply_motor_unblocked(responder->match_tx, responder->emit_feedback,
					   &responder->cursor);
		k_sleep(K_MSEC(1));
	}

	keep_reply_motor_unblocked(responder->match_tx, responder->emit_feedback,
				   &responder->cursor);
}

static void reply_responder_start(struct reply_responder *responder,
				  bool (*match_tx)(const struct can_frame *frame),
				  void (*emit_feedback)(void), uint32_t cursor)
{
	*responder = (struct reply_responder){
		.match_tx = match_tx,
		.emit_feedback = emit_feedback,
		.cursor = cursor,
	};
	k_thread_create(&reply_responder_thread, reply_responder_stack,
			K_THREAD_STACK_SIZEOF(reply_responder_stack), reply_responder_entry,
			responder, NULL, NULL, K_PRIO_PREEMPT(0), 0, K_NO_WAIT);
}

static void reply_responder_stop(struct reply_responder *responder)
{
	responder->stop = true;
	k_thread_join(&reply_responder_thread, K_MSEC(100));
}

static void drain_reply_motor(bool (*match_tx)(const struct can_frame *frame),
			      void (*emit_feedback)(void), int drain_ms)
{
	uint32_t cursor = sim_current_tx_count();
	int64_t deadline = k_uptime_get() + drain_ms;

	while (k_uptime_get() < deadline) {
		keep_reply_motor_unblocked(match_tx, emit_feedback, &cursor);
		k_sleep(K_MSEC(1));
	}
}

static void drain_all_reply_motors_until_quiet(int quiet_ms, int timeout_ms)
{
	uint32_t dm_cursor = sim_current_tx_count();
	uint32_t mi_cursor = dm_cursor;
	uint32_t rs_cursor = dm_cursor;
	uint32_t lk_cursor = dm_cursor;
	int64_t deadline = k_uptime_get() + timeout_ms;
	int64_t quiet_since = k_uptime_get();

	while (k_uptime_get() < deadline) {
		bool replied = false;

		replied |= keep_reply_motor_unblocked(match_dm_tx, emit_dm_feedback, &dm_cursor);
		replied |= keep_reply_motor_unblocked(match_mi_tx, emit_mi_feedback, &mi_cursor);
		replied |= keep_reply_motor_unblocked(match_rs_tx, emit_rs_feedback, &rs_cursor);
		replied |= keep_reply_motor_unblocked(match_lk_tx, emit_lk_feedback, &lk_cursor);

		if (replied) {
			quiet_since = k_uptime_get();
		} else if (k_uptime_get() - quiet_since >= quiet_ms) {
			return;
		}
		k_sleep(K_MSEC(1));
	}
}

static uint32_t count_reply_motor_rate(bool (*match_tx)(const struct can_frame *frame),
				       void (*emit_feedback)(void), int window_ms)
{
	uint32_t start = sim_current_tx_count();
	uint32_t cursor = start;
	int64_t deadline = k_uptime_get() + window_ms;

	while (k_uptime_get() < deadline) {
		keep_reply_motor_unblocked(match_tx, emit_feedback, &cursor);
		k_sleep(K_MSEC(1));
	}

	return sim_count_tx_since(start, match_tx);
}

static void expect_rate_window(const char *name, uint32_t count, uint32_t min, uint32_t max)
{
	if (count < min || count > max) {
		sim_dump_tx_since(0);
	}
	zassert_between_inclusive(count, min, max,
				  "%s sent %u frames outside expected rate window [%u, %u]", name,
				  count, min, max);
}

typedef void (*expected_payload_fn)(float value, uint8_t data[CAN_MAX_DLEN]);

static void bring_request_reply_motor_online(const struct device *dev, const char *name,
					     bool (*match_tx)(const struct can_frame *frame),
					     void (*emit_feedback)(void))
{
	uint32_t tx_start;

	driver_motor_control(dev, DISABLE_MOTOR);
	k_sleep(K_MSEC(15));
	driver_motor_control(dev, ENABLE_MOTOR);
	tx_start = sim_current_tx_count();
	wait_for_tx_after(tx_start, match_tx, name);
	emit_feedback();
	wait_for_online_state(dev, true, ONLINE_RECOVERY_MS, name);
}

static void verify_mit_payload_sequence(const struct device *dev, const char *name,
					bool (*match_tx)(const struct can_frame *frame),
					void (*emit_feedback)(void),
					expected_payload_fn build_payload, uint32_t expected_id)
{
	const float speeds[] = {10.0f, 20.0f, 30.0f};
	uint8_t payload[CAN_MAX_DLEN];
	uint32_t start;
	uint32_t previous_match = UINT32_MAX;

	driver_motor_control(dev, DISABLE_MOTOR);
	drain_all_reply_motors_until_quiet(50, 1200);
	sim_reset_tx_history();
	start = sim_current_tx_count();
	driver_motor_control(dev, ENABLE_MOTOR);

	for (int i = 0; i < ARRAY_SIZE(speeds); i++) {
		uint32_t set_at_ms = (uint32_t)k_uptime_get();
		struct reply_responder responder;
		int ret;

		reply_responder_start(&responder, match_tx, emit_feedback, start);
		ret = motor_set_mit(dev, speeds[i], 0.0f, 0.0f);
		reply_responder_stop(&responder);

		zassert_equal(ret, 0, "%s rejected MIT setpoint %d", name, ret);
		build_payload(speeds[i], payload);
		expect_payload_sequence_step(name, match_tx, expected_id, CAN_EXT_ID_MASK, payload,
					     set_at_ms, emit_feedback, CONTROL_LATENCY_MS, &start,
					     &previous_match);
		wait_for_online_state(dev, true, ONLINE_RECOVERY_MS, name);
	}

	driver_motor_control(dev, DISABLE_MOTOR);
	drain_all_reply_motors_until_quiet(50, 1200);
}

static void verify_lk_payload_sequence(void)
{
	const float speeds[] = {25.0f, 50.0f, 75.0f};
	uint8_t payload[CAN_MAX_DLEN];
	uint32_t start;
	uint32_t previous_match = UINT32_MAX;

	driver_motor_control(LK_DEV, DISABLE_MOTOR);
	drain_reply_motor(match_lk_tx, emit_lk_feedback, 600);
	sim_reset_tx_history();
	start = sim_current_tx_count();
	driver_motor_control(LK_DEV, ENABLE_MOTOR);
	drain_reply_motor(match_lk_tx, emit_lk_feedback, 1300);
	sim_reset_tx_history();
	start = sim_current_tx_count();

	for (int i = 0; i < ARRAY_SIZE(speeds); i++) {
		uint32_t set_at_ms = (uint32_t)k_uptime_get();
		int ret = motor_set_speed(LK_DEV, speeds[i]);

		zassert_equal(ret, 0, "LK rejected speed setpoint %d", ret);
		lk_tx_data_handler(NULL);
		expected_lk_speed(speeds[i], payload);
		expect_payload_sequence_step("LK", match_lk_tx, 0x140U + LK_ID, CAN_STD_ID_MASK,
					     payload, set_at_ms, emit_lk_feedback,
					     CONTROL_LATENCY_MS, &start, &previous_match);
		wait_for_online_state(LK_DEV, true, ONLINE_RECOVERY_MS, "LK");
	}

	driver_motor_control(LK_DEV, DISABLE_MOTOR);
	drain_reply_motor(match_lk_tx, emit_lk_feedback, 600);
}

static void verify_dji_pid_sequence(void)
{
	const float targets[] = {1000.0f, 1500.0f, 2000.0f};
	uint8_t payload[CAN_MAX_DLEN];
	uint32_t start;
	uint32_t previous_match = UINT32_MAX;

	driver_motor_control(DJI_DEV, DISABLE_MOTOR);
	configure_dji_speed_test_limits(DJI_DEV);
	driver_motor_control(DJI_DEV, ENABLE_MOTOR);
	k_sleep(K_MSEC(5));
	sim_reset_tx_history();
	start = sim_current_tx_count();

	for (int i = 0; i < ARRAY_SIZE(targets); i++) {
		int16_t expected_current;
		uint32_t report_at_ms;
		int ret = motor_set_speed(DJI_DEV, targets[i]);

		zassert_equal(ret, 0, "DJI rejected speed setpoint %d", ret);
		expected_current = dji_expected_current(targets[i], 0.0f);
		expected_dji_current(expected_current, payload);

		k_sleep(K_MSEC(1));
		report_at_ms = (uint32_t)k_uptime_get();
		emit_dji_report_with_rpm(0);
		service_dji_tx_work();
		expect_payload_sequence_step("DJI", match_dji_tx, DJI_TX_ID, CAN_STD_ID_MASK,
					     payload, report_at_ms, NULL, DJI_CONTROL_LATENCY_MS,
					     &start, &previous_match);
	}
}

ZTEST(motor_driver_sim, test_dji_same_tx_id_packs_multiple_motors)
{
	uint8_t payload[CAN_MAX_DLEN] = {0};
	uint32_t start;
	uint32_t previous_match = UINT32_MAX;
	int16_t current0 = dji_expected_current(1000.0f, 0.0f);
	int16_t current1 = dji_expected_current(500.0f, 0.0f);
	uint32_t report_at_ms;

	driver_motor_control(DJI_DEV, ENABLE_MOTOR);
	driver_motor_control(DJI2_DEV, ENABLE_MOTOR);
	configure_dji_speed_test_limits(DJI_DEV);
	configure_dji_speed_test_limits(DJI2_DEV);
	zassert_equal(motor_set_speed(DJI_DEV, 1000.0f), 0, "DJI0 rejected speed setpoint");
	zassert_equal(motor_set_speed(DJI2_DEV, 500.0f), 0, "DJI1 rejected speed setpoint");

	sim_reset_tx_history();
	start = sim_current_tx_count();
	emit_dji_report_for(DJI_RX_ID, 0);
	service_dji_tx_work();
	zassert_false(sim_tx_seen_since(start, match_dji_tx),
		      "DJI same-tx frame sent before all motor reports arrived");

	expected_dji_current_slot(current0, 0, payload);
	expected_dji_current_slot(current1, 1, payload);
	report_at_ms = (uint32_t)k_uptime_get();
	emit_dji_report_for(DJI2_RX_ID, 0);
	service_dji_tx_work();
	expect_payload_sequence_step("DJI same tx", match_dji_tx, DJI_TX_ID, CAN_STD_ID_MASK,
				     payload, report_at_ms, NULL, DJI_CONTROL_LATENCY_MS, &start,
				     &previous_match);
}

ZTEST(motor_driver_sim, test_dji_distinct_tx_ids_send_independent_frames)
{
	uint8_t payload0[CAN_MAX_DLEN] = {0};
	uint8_t payload2[CAN_MAX_DLEN] = {0};
	uint32_t start;
	uint32_t previous_match = UINT32_MAX;
	uint32_t report_at_ms;

	driver_motor_control(DJI_DEV, ENABLE_MOTOR);
	driver_motor_control(DJI3_DEV, ENABLE_MOTOR);
	configure_dji_speed_test_limits(DJI_DEV);
	configure_dji_speed_test_limits(DJI3_DEV);
	zassert_equal(motor_set_speed(DJI_DEV, 1000.0f), 0, "DJI0 rejected speed setpoint");
	zassert_equal(motor_set_speed(DJI3_DEV, 700.0f), 0, "DJI2 rejected speed setpoint");

	sim_reset_tx_history();
	start = sim_current_tx_count();

	expected_dji_current_slot(dji_expected_current(1000.0f, 0.0f), 0, payload0);
	report_at_ms = (uint32_t)k_uptime_get();
	emit_dji_report_for(DJI_RX_ID, 0);
	service_dji_tx_work();
	expect_payload_sequence_step("DJI tx 0x200", match_dji_tx, DJI_TX_ID, CAN_STD_ID_MASK,
				     payload0, report_at_ms, NULL, DJI_CONTROL_LATENCY_MS, &start,
				     &previous_match);

	previous_match = UINT32_MAX;
	expected_dji_current_slot(dji_expected_current(700.0f, 0.0f), 2, payload2);
	report_at_ms = (uint32_t)k_uptime_get();
	emit_dji_report_for(DJI3_RX_ID, 0);
	service_dji_tx_work();
	expect_payload_sequence_step("DJI tx 0x1ff", match_dji3_tx, DJI3_TX_ID, CAN_STD_ID_MASK,
				     payload2, report_at_ms, NULL, DJI_CONTROL_LATENCY_MS, &start,
				     &previous_match);
}

static void *motor_driver_sim_setup(void)
{
	k_sleep(K_MSEC(1300));
	sim_reset_tx_history();
	return NULL;
}

static void motor_driver_sim_before(void *fixture)
{
	ARG_UNUSED(fixture);

	driver_motor_control(DM_DEV, DISABLE_MOTOR);
	driver_motor_control(MI_DEV, DISABLE_MOTOR);
	driver_motor_control(RS_DEV, DISABLE_MOTOR);
	driver_motor_control(LK_DEV, DISABLE_MOTOR);
	driver_motor_control(DJI_DEV, DISABLE_MOTOR);
	driver_motor_control(DJI2_DEV, DISABLE_MOTOR);
	driver_motor_control(DJI3_DEV, DISABLE_MOTOR);
	drain_all_reply_motors_until_quiet(50, 1200);
	sim_reset_tx_history();
}

static void verify_requested_enabled_edges(const struct device *dev, const char *name)
{
	driver_motor_control(dev, DISABLE_MOTOR);
	expect_requested_enabled(dev, false, name);
	driver_motor_control(dev, ENABLE_MOTOR);
	expect_requested_enabled(dev, true, name);
	driver_motor_control(dev, DISABLE_MOTOR);
	expect_requested_enabled(dev, false, name);
}

static void verify_status_enabled_mirrors_request(const struct device *dev, const char *name)
{
	driver_motor_control(dev, DISABLE_MOTOR);
	expect_status_enabled(dev, false, name);
	driver_motor_control(dev, ENABLE_MOTOR);
	expect_requested_enabled(dev, true, name);
	expect_status_enabled(dev, true, name);
	driver_motor_control(dev, DISABLE_MOTOR);
	expect_requested_enabled(dev, false, name);
	expect_status_enabled(dev, false, name);
}

static void verify_rs_status_enabled_from_feedback(void)
{
	driver_motor_control(RS_DEV, DISABLE_MOTOR);
	force_motor_offline(RS_DEV);
	expect_status_enabled(RS_DEV, false, "RS");

	driver_motor_control(RS_DEV, ENABLE_MOTOR);
	expect_requested_enabled(RS_DEV, true, "RS");
	emit_rs_reset_feedback();
	wait_for_online_state(RS_DEV, true, ONLINE_RECOVERY_MS, "RS");
	expect_status_enabled(RS_DEV, false, "RS");

	emit_rs_feedback();
	expect_status_enabled(RS_DEV, true, "RS");

	driver_motor_control(RS_DEV, DISABLE_MOTOR);
	expect_requested_enabled(RS_DEV, false, "RS");
	expect_status_enabled(RS_DEV, true, "RS");
	emit_rs_reset_feedback();
	expect_status_enabled(RS_DEV, false, "RS");
}

static void verify_disable_preserves_online(const struct device *dev, const char *name,
					    void (*emit_feedback)(void))
{
	driver_motor_control(dev, DISABLE_MOTOR);
	force_motor_offline(dev);
	driver_motor_control(dev, ENABLE_MOTOR);
	emit_feedback();
	wait_for_online_state(dev, true, ONLINE_RECOVERY_MS, name);

	driver_motor_control(dev, DISABLE_MOTOR);
	expect_requested_enabled(dev, false, name);
	expect_online(dev, true, name);
}

static void verify_clear_error_requested_policy(const struct device *dev, const char *name,
						bool expected_requested_enabled)
{
	driver_motor_control(dev, DISABLE_MOTOR);
	driver_motor_control(dev, ENABLE_MOTOR);
	expect_requested_enabled(dev, true, name);
	driver_motor_control(dev, CLEAR_ERROR);
	expect_requested_enabled(dev, expected_requested_enabled, name);
	driver_motor_control(dev, DISABLE_MOTOR);
}

ZTEST(motor_driver_sim, test_request_reply_motor_drivers)
{
	verify_request_reply_motor(DM_DEV, "DM", match_dm_tx, emit_dm_feedback, 500);
	verify_request_reply_motor(MI_DEV, "MI", match_mi_tx, emit_mi_feedback, 6000);
	verify_request_reply_motor(RS_DEV, "RS", match_rs_tx, emit_rs_feedback, 0);
	verify_request_reply_motor(LK_DEV, "LK", match_lk_tx, emit_lk_feedback, 3000);
}

ZTEST(motor_driver_sim, test_requested_enabled_state_transitions)
{
	verify_requested_enabled_edges(DM_DEV, "DM");
	verify_requested_enabled_edges(MI_DEV, "MI");
	verify_requested_enabled_edges(RS_DEV, "RS");
	verify_requested_enabled_edges(LK_DEV, "LK");
	verify_requested_enabled_edges(DJI_DEV, "DJI");
	verify_requested_enabled_edges(DJI2_DEV, "DJI2");
	verify_requested_enabled_edges(DJI3_DEV, "DJI3");

	drain_all_reply_motors_until_quiet(50, 1200);
}

ZTEST(motor_driver_sim, test_status_enabled_state_transitions)
{
	verify_status_enabled_mirrors_request(MI_DEV, "MI");
	verify_rs_status_enabled_from_feedback();
	verify_status_enabled_mirrors_request(LK_DEV, "LK");
	verify_status_enabled_mirrors_request(DJI_DEV, "DJI");

	driver_motor_control(DM_DEV, DISABLE_MOTOR);
	emit_dm_feedback_enabled(false);
	expect_requested_enabled(DM_DEV, false, "DM");
	expect_status_enabled(DM_DEV, false, "DM");
	expect_online(DM_DEV, true, "DM");

	driver_motor_control(DM_DEV, ENABLE_MOTOR);
	expect_requested_enabled(DM_DEV, true, "DM");
	expect_status_enabled(DM_DEV, false, "DM");
	emit_dm_feedback_enabled(true);
	expect_status_enabled(DM_DEV, true, "DM");

	driver_motor_control(DM_DEV, DISABLE_MOTOR);
	expect_requested_enabled(DM_DEV, false, "DM");
	expect_status_enabled(DM_DEV, true, "DM");
	emit_dm_feedback_enabled(false);
	expect_status_enabled(DM_DEV, false, "DM");

	drain_all_reply_motors_until_quiet(50, 1200);
}

ZTEST(motor_driver_sim, test_disable_preserves_online_state)
{
	verify_disable_preserves_online(DM_DEV, "DM", emit_dm_feedback);
	verify_disable_preserves_online(MI_DEV, "MI", emit_mi_feedback);
	verify_disable_preserves_online(RS_DEV, "RS", emit_rs_feedback);
	verify_disable_preserves_online(LK_DEV, "LK", emit_lk_feedback);
	verify_disable_preserves_online(DJI_DEV, "DJI", emit_dji_report);

	drain_all_reply_motors_until_quiet(50, 1200);
}

ZTEST(motor_driver_sim, test_clear_error_requested_enabled_policy)
{
	verify_clear_error_requested_policy(DM_DEV, "DM", true);
	verify_clear_error_requested_policy(MI_DEV, "MI", true);
	verify_clear_error_requested_policy(LK_DEV, "LK", true);
	verify_clear_error_requested_policy(DJI_DEV, "DJI", true);
	verify_clear_error_requested_policy(RS_DEV, "RS", false);

	drain_all_reply_motors_until_quiet(50, 1200);
}

ZTEST(motor_driver_sim, test_rs_offline_enable_retries_auto_report)
{
	driver_motor_control(RS_DEV, DISABLE_MOTOR);
	force_motor_offline(RS_DEV);
	driver_motor_control(RS_DEV, ENABLE_MOTOR);
	k_sleep(K_MSEC(20));
	sim_reset_tx_history();

	wait_for_tx_after(0, match_rs_enable_tx, "RS offline enable retry");
	wait_for_tx_after(0, match_rs_auto_report_tx, "RS offline auto-report retry");
}

ZTEST(motor_driver_sim, test_rs_online_reset_state_retries_enable)
{
	driver_motor_control(RS_DEV, DISABLE_MOTOR);
	force_motor_offline(RS_DEV);
	driver_motor_control(RS_DEV, ENABLE_MOTOR);
	emit_rs_reset_feedback();
	wait_for_online_state(RS_DEV, true, ONLINE_RECOVERY_MS, "RS");
	expect_requested_enabled(RS_DEV, true, "RS");
	expect_status_enabled(RS_DEV, false, "RS");

	sim_reset_tx_history();
	wait_for_tx_after(0, match_rs_enable_tx, "RS online reset enable retry");
}

ZTEST(motor_driver_sim, test_continuous_command_packing_order_and_latency)
{
	verify_mit_payload_sequence(DM_DEV, "DM", match_dm_tx, emit_dm_feedback, expected_dm_mit,
				    DM_TX_ID);
	verify_mit_payload_sequence(MI_DEV, "MI", match_mi_tx, emit_mi_feedback, expected_mi_mit,
				    expected_mi_mit_id());
	verify_mit_payload_sequence(RS_DEV, "RS", match_rs_tx, emit_rs_feedback, expected_rs_mit,
				    expected_rs_mit_id());
	verify_lk_payload_sequence();
}

ZTEST(motor_driver_sim, test_control_send_rate_windows)
{
	uint32_t count;

	driver_motor_control(DJI_DEV, ENABLE_MOTOR);
	motor_set_speed(DJI_DEV, 1000.0f);
	sim_reset_tx_history();
	int64_t deadline = k_uptime_get() + DJI_RATE_WINDOW_MS;

	while (k_uptime_get() < deadline) {
		emit_dji_report_with_rpm(0);
		service_dji_tx_work();
		k_sleep(K_MSEC(4));
	}
	count = sim_count_tx_since(0, match_dji_tx);
	expect_rate_window("DJI", count, 8, 100);

	bring_request_reply_motor_online(DM_DEV, "DM", match_dm_tx, emit_dm_feedback);
	drain_reply_motor(match_dm_tx, emit_dm_feedback, 40);
	count = count_reply_motor_rate(match_dm_tx, emit_dm_feedback, DM_RATE_WINDOW_MS);
	expect_rate_window("DM", count, 8, 50);
	driver_motor_control(DM_DEV, DISABLE_MOTOR);
	drain_all_reply_motors_until_quiet(50, 1200);
	sim_reset_tx_history();

	bring_request_reply_motor_online(MI_DEV, "MI", match_mi_tx, emit_mi_feedback);
	drain_reply_motor(match_mi_tx, emit_mi_feedback, 40);
	count = count_reply_motor_rate(match_mi_tx, emit_mi_feedback, MI_RATE_WINDOW_MS);
	expect_rate_window("MI", count, 6, 80);
	driver_motor_control(MI_DEV, DISABLE_MOTOR);
	drain_all_reply_motors_until_quiet(50, 1200);
	sim_reset_tx_history();
}

ZTEST(motor_driver_sim, test_dji_control_not_starved_by_reply_backlog)
{
	uint32_t start;

	driver_motor_control(DM_DEV, ENABLE_MOTOR);
	wait_for_tx_after(sim_current_tx_count(), match_dm_tx, "DM");
	k_sleep(K_MSEC(80));

	driver_motor_control(DJI_DEV, ENABLE_MOTOR);
	start = sim_current_tx_count();
	motor_set_speed(DJI_DEV, 1000.0f);
	emit_dji_report_with_rpm(0);
	service_dji_tx_work();
	wait_for_tx_after(start, match_dji_tx, "DJI");

	driver_motor_control(DM_DEV, DISABLE_MOTOR);
	drain_reply_motor(match_dm_tx, emit_dm_feedback, 40);
}

ZTEST(motor_driver_sim, test_a_dji_pid_response)
{
	verify_dji_pid_sequence();
}

ZTEST(motor_driver_sim, test_a_dji_periodic_report_driver)
{
	uint32_t tx_start;
	uint32_t recovery_ms;

	driver_motor_control(DJI_DEV, DISABLE_MOTOR);
	k_sleep(K_MSEC(5));
	driver_motor_control(DJI_DEV, ENABLE_MOTOR);

	tx_start = sim_current_tx_count();
	emit_dji_report();
	service_dji_tx_work();
	recovery_ms = wait_for_online_state_timed(DJI_DEV, true, ONLINE_RECOVERY_MS, "DJI");
	zassert_true(recovery_ms <= ONLINE_RECOVERY_MS, "DJI recovery took %u ms", recovery_ms);
	wait_for_tx_after(tx_start, match_dji_tx, "DJI");

	k_sleep(K_MSEC(30));
	expect_online(DJI_DEV, false, "DJI");

	tx_start = sim_current_tx_count();
	emit_dji_report();
	service_dji_tx_work();
	recovery_ms = wait_for_online_state_timed(DJI_DEV, true, ONLINE_RECOVERY_MS, "DJI");
	zassert_true(recovery_ms <= ONLINE_RECOVERY_MS, "DJI recovery took %u ms", recovery_ms);
	wait_for_tx_after(tx_start, match_dji_tx, "DJI");
}

ZTEST_SUITE(motor_driver_sim, NULL, motor_driver_sim_setup, motor_driver_sim_before, NULL, NULL);
