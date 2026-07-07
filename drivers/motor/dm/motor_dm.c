/*
 * Copyright (c) 2024 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "../common/common.h"
#include "../common/motor_link.h"
#include "../common/motor_can_sched.h"
#include "motor_dm.h"
#include "syscalls/kernel.h"
#include "zephyr/drivers/can.h"
#include "zephyr/drivers/motor.h"
#include "zephyr/kernel.h"
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT dm_motor

#define DM_OFFLINE_ENABLE_RETRY_TX_COUNT 50U
#define DM_OFFLINE_MISSED_REPLIES        4
#define DM_TX_TIMER_PERIOD_MS            1U

#define DM_REG_RUN_MODE 0x0A
#define DM_REG_KP_ASR   0x19
#define DM_REG_KI_ASR   0x1A
#define DM_REG_KP_APR   0x1B
#define DM_REG_KI_APR   0x1C

LOG_MODULE_REGISTER(motor_dm, CONFIG_MOTOR_LOG_LEVEL);

/**
 * @brief Converts an unsigned integer to a float, given range and number of bits.
 *
 * This function takes an unsigned integer and maps it to a float value within
 * the specified range [x_min, x_max] based on the number of bits.
 *
 * @param x_int The unsigned integer value to be converted.
 * @param x_min The minimum value of the target float range.
 * @param x_max The maximum value of the target float range.
 * @param bits The number of bits representing the unsigned integer.
 * @return The corresponding float value within the range [x_min, x_max].
 */

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/// converts unsigned int to float, given range and number of bits ///
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief Converts a float to an unsigned int, given range and number of bits.
 *
 * This function takes a floating-point number and converts it to an unsigned
 * integer representation based on the specified range and number of bits.
 *
 * @param x The floating-point number to convert.
 * @param x_min The minimum value of the range.
 * @param x_max The maximum value of the range.
 * @param bits The number of bits for the unsigned integer representation.
 *
 * @return The unsigned integer representation of the floating-point number.
 */
static inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
	/// Converts a float to an unsigned int, given range and number of bits
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static uint32_t dm_control_period_ms(const struct dm_motor_config *cfg)
{
	uint32_t freq = cfg->freq > 0 ? (uint32_t)cfg->freq : 1U;

	return MAX(1U, 1000U / freq);
}

static int dm_send_cmd_frame(const struct device *dev, const uint8_t data[8], const char *tag)
{
	struct dm_motor_data *motor_data = dev->data;
	const struct dm_motor_config *cfg = dev->config;
	struct can_frame frame = {
		.id = cfg->common.tx_id + motor_data->tx_offset,
		.flags = 0,
		.dlc = 8,
	};

	memcpy(frame.data, data, 8);
	return motor_can_sched_send_with_priority(cfg->common.phy, &frame,
						  MOTOR_CAN_SCHED_PRIO_CRITICAL, tag);
}

static int dm_send_cmd_frame_reply(const struct device *dev, const uint8_t data[8], const char *tag)
{
	struct dm_motor_data *motor_data = dev->data;
	const struct dm_motor_config *cfg = dev->config;
	struct can_frame frame = {
		.id = cfg->common.tx_id + motor_data->tx_offset,
		.flags = 0,
		.dlc = 8,
	};

	memcpy(frame.data, data, 8);
	return motor_can_sched_send_reply(cfg->common.phy, &frame, cfg->common.rx_id & 0xFF,
					  CAN_STD_ID_MASK, 5U, tag);
}

int dm_init(const struct device *dev)
{
	const struct dm_motor_config *cfg = dev->config;
	static bool work_queue_started;

	if (!device_is_ready(cfg->common.phy)) {
		return -1;
	}
	if (k_work_busy_get(&dm_init_work) != 0) {
		return 0;
	}
	if (!work_queue_started) {
		k_work_queue_init(&dm_work_queue);
		k_work_queue_start(&dm_work_queue, dm_work_queue_stack, CAN_SEND_STACK_SIZE,
				   CAN_SEND_PRIORITY, NULL);
		work_queue_started = true;
	}

	dm_tx_timer.expiry_fn = dm_isr_init_handler;
	k_timer_start(&dm_tx_timer, K_MSEC(500), K_MSEC(7));
	k_timer_user_data_set(&dm_tx_timer, &dm_init_work);
	return 0;
}

void dm_control(const struct device *dev, enum motor_cmd cmd)
{
	struct dm_motor_data *data = dev->data;

	switch (cmd) {
	case ENABLE_MOTOR:
		if (dm_send_cmd_frame_reply(dev, enable_frame, "dm-enable") < 0) {
			motor_stats_inc(MOTOR_STAT_TX_ERROR);
		}
		motor_link_request_enable(&data->common.link);
		data->tx_cnt = 0;
		data->last_tx_time = 0;
		break;
	case DISABLE_MOTOR:
		if (dm_send_cmd_frame_reply(dev, disable_frame, "dm-disable") < 0) {
			motor_stats_inc(MOTOR_STAT_TX_ERROR);
		}
		motor_link_request_disable(&data->common.link);
		data->tx_cnt = 0;
		break;
	case SET_ZERO:
		if (dm_send_cmd_frame(dev, set_zero_frame, "dm-set-zero") < 0) {
			motor_stats_inc(MOTOR_STAT_TX_ERROR);
		}
		break;
	case CLEAR_CONTROLLER:
		memset(&data->params, 0, sizeof(data->params));
		break;
	case CLEAR_ERROR:
		if (dm_send_cmd_frame(dev, clear_error_frame, "dm-clear-error") < 0) {
			motor_stats_inc(MOTOR_STAT_TX_ERROR);
		}
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_CMD);
		break;
	}
}

static void dm_motor_pack(const struct device *dev, struct can_frame *frame)
{
	uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
	uint8_t *pbuf, *vbuf;
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	frame->id = cfg->common.tx_id + data->tx_offset;
	frame->dlc = 8;
	frame->flags = 0;
	switch (data->common.mode) {
	case MIT:
		pos_tmp = float_to_uint(data->target_angle, -cfg->p_max, cfg->p_max, 16);
		vel_tmp = float_to_uint(data->target_radps, -cfg->v_max, cfg->v_max, 12);
		tor_tmp = float_to_uint(data->target_torque, -cfg->t_max, cfg->t_max, 12);
		kp_tmp = float_to_uint(data->params.k_p, 0, 500, 12);
		kd_tmp = float_to_uint(data->params.k_d, 0, 5, 12);

		frame->data[0] = (pos_tmp >> 8);
		frame->data[1] = pos_tmp;
		frame->data[2] = (vel_tmp >> 4);
		frame->data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
		frame->data[4] = kp_tmp;
		frame->data[5] = (kd_tmp >> 4);
		frame->data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
		frame->data[7] = tor_tmp;
		break;
	case PV:
		pbuf = (uint8_t *)&data->target_angle;
		vbuf = (uint8_t *)&data->target_radps;

		memcpy(frame->data, pbuf, 4);
		memcpy(frame->data + 4, vbuf, 4);
		break;
	case VO:
		vbuf = (uint8_t *)&data->target_radps;

		memcpy(frame->data, vbuf, 4);
		break;
	default:
		break;
	}
}

static int dm_send_control_frame(const struct device *dev)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;
	struct can_frame tx_frame = {0};
	int ret;

	dm_motor_pack(dev, &tx_frame);
	ret = motor_can_sched_send_reply(cfg->common.phy, &tx_frame, cfg->common.rx_id & 0xFF,
					 CAN_STD_ID_MASK, 5U, "dm-control");
	if (ret == 0) {
		data->last_tx_time = k_uptime_get();
		data->tx_cnt++;
	} else {
		motor_stats_inc(MOTOR_STAT_TX_ERROR);
	}
	return ret;
}

int dm_get(const struct device *dev, motor_status_t *status)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;

	status->angle = data->common.angle;
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;

	status->mode = data->common.mode;
	status->target = data->common.target;
	status->controller_id = data->common.controller_id;
	status->sum_angle = data->delta_deg_sum;
	status->speed_limit[0] = cfg->v_max;
	status->speed_limit[1] = cfg->v_max;
	status->torque_limit[0] = cfg->t_max;
	status->torque_limit[1] = cfg->t_max;
	status->online = data->common.link.online;
	status->enabled = data->enabled;
	status->error = data->err;
	return 0;
}

static void dm_rx_handler(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
	const struct device *dev = user_data;
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;
	uint8_t motor_id = frame->data[0] & 0x0F;

	if (motor_id != (cfg->common.tx_id & 0x0F)) {
		return;
	}

	motor_can_sched_report_rx(can_dev, frame);
	data->prev_recv_time = k_uptime_get_32();

	data->err = frame->data[0] >> 4;
	data->enabled = data->err & 1;
	data->RAWangle = (frame->data[1] << 8) | (frame->data[2]);
	data->RAWrpm = (frame->data[3] << 4) | (frame->data[4] >> 4);
	data->RAWtorque = (frame->data[4] & 0xF) << 8;
	data->update = true;

	motor_link_observe_reply(dev, &data->common.link);

	k_work_submit_to_queue(&dm_work_queue, &dm_rx_data_handle);
}

static void dm_edit_reg_value(const struct device *dev, uint16_t can_id, uint8_t reg_addr,
			      uint32_t reg_value)
{
	struct can_frame frame;
	frame.id = 0x7FF;
	frame.dlc = 8;
	frame.flags = 0;
	frame.data[CANID_L] = can_id & 0xFF;
	frame.data[CANID_H] = can_id >> 8;
	frame.data[2] = 0x55;
	frame.data[RID] = reg_addr;
	frame.data[4] = reg_value & 0xFF;
	frame.data[5] = reg_value >> 8;
	frame.data[6] = reg_value >> 16;
	frame.data[7] = reg_value >> 24;
	motor_can_sched_send_with_priority(dev, &frame, MOTOR_CAN_SCHED_PRIO_CRITICAL, "dm-reg");
}

static void dm_edit_reg_float(const struct device *dev, uint16_t can_id, uint8_t reg_addr,
			      float reg_value)
{
	union {
		float f;
		uint32_t u;
	} conv = {.f = reg_value};

	dm_edit_reg_value(dev, can_id, reg_addr, conv.u);
}

static void dm_apply_controller_mode(const struct device *dev, enum motor_mode mode)
{
	struct dm_motor_data *data = dev->data;
	const struct dm_motor_config *cfg = dev->config;
	struct motor_controller_params params = {0};
	struct motor_controller_params pv_velocity_params = {0};
	char mode_str[10];

	data->common.mode = mode;
	data->common.target = mode == VO ? MOTOR_TARGET_SPEED : MOTOR_TARGET_POSITION;

	switch (mode) {
	case MIT:
		snprintf(mode_str, sizeof(mode_str), "mit");
		data->tx_offset = 0x0;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, DM_REG_RUN_MODE, 0x01);
		break;
	case PV:
		snprintf(mode_str, sizeof(mode_str), "pv");
		data->tx_offset = 0x100;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, DM_REG_RUN_MODE, 0x02);
		break;
	case VO:
		snprintf(mode_str, sizeof(mode_str), "vo");
		data->tx_offset = 0x200;
		dm_edit_reg_value(cfg->common.phy, cfg->common.tx_id, DM_REG_RUN_MODE, 0x03);
		break;
	default:
		dm_control(dev, DISABLE_MOTOR);
	}

	bool found = false;
	for (int i = 0; i < motor_get_controller_count(dev); i++) {
		const struct motor_controller_config *ctrl_cfg = &cfg->common.controllers[i];

		if (ctrl_cfg->param_count == 0) {
			break;
		}
		if (data->common.controller_id != MOTOR_CONTROLLER_ID_AUTO &&
		    data->common.controller_id != i) {
			continue;
		}
		if (strcmp(ctrl_cfg->info.name, mode_str) == 0 || ctrl_cfg->info.mode == mode) {
			if (motor_controller_get_params(ctrl_cfg, 0, &params) < 0) {
				continue;
			}
			if (mode == PV &&
			    motor_controller_get_params(ctrl_cfg, 1, &pv_velocity_params) < 0) {
				continue;
			}

			data->common.mode = mode;
			data->common.controller_id = i;
			data->params = params;
			found = true;
			break;
		}
	}
	if (!found) {
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		if (mode != VO) {
			dm_control(dev, DISABLE_MOTOR);
		}
	} else if (mode == VO) {
		dm_edit_reg_float(cfg->common.phy, cfg->common.tx_id, DM_REG_KP_ASR,
				  data->params.k_p);
		dm_edit_reg_float(cfg->common.phy, cfg->common.tx_id, DM_REG_KI_ASR,
				  data->params.k_i);
	} else if (mode == PV) {
		dm_edit_reg_float(cfg->common.phy, cfg->common.tx_id, DM_REG_KP_ASR,
				  pv_velocity_params.k_p);
		dm_edit_reg_float(cfg->common.phy, cfg->common.tx_id, DM_REG_KI_ASR,
				  pv_velocity_params.k_i);
		dm_edit_reg_float(cfg->common.phy, cfg->common.tx_id, DM_REG_KP_APR, params.k_p);
		dm_edit_reg_float(cfg->common.phy, cfg->common.tx_id, DM_REG_KI_APR, params.k_i);
	}
}

int dm_set(const struct device *dev, motor_setpoint_t *status)
{
	struct dm_motor_data *data = dev->data;
	motor_controller_info_t controller = {0};
	enum motor_mode previous_mode = data->common.mode;
	uint8_t previous_controller_id = data->common.controller_id;
	int ret;

	ret = motor_resolve_controller(dev, status, &controller);
	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		return ret;
	}
	if (status->target != MOTOR_TARGET_NONE) {
		data->common.controller_id = status->controller_id;
	} else {
		return 0;
	}

	if (status->mode == MIT) {
		data->common.target = MOTOR_TARGET_POSITION;
		data->target_angle = status->angle / (RAD2DEG);
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_torque = status->torque;
		// data->params.k_p = 0;
		// data->params.k_d = 0;
	} else if (status->mode == PV) {
		if (status->target != MOTOR_TARGET_NONE &&
		    status->target != MOTOR_TARGET_POSITION) {
			return -ENOSYS;
		}
		data->common.target = MOTOR_TARGET_POSITION;
		data->target_angle = status->angle / (RAD2DEG);
		data->target_radps = RPM2RADPS(status->rpm);
	} else if (status->mode == VO) {
		if (status->target != MOTOR_TARGET_NONE && status->target != MOTOR_TARGET_SPEED) {
			return -ENOSYS;
		}
		data->common.target = MOTOR_TARGET_SPEED;
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_angle = 0;
		data->target_torque = 0;
		data->params.k_p = 0;
		data->params.k_d = 0;
	} else {
		return -ENOSYS;
	}

	if (previous_mode != status->mode || previous_controller_id != status->controller_id) {
		dm_apply_controller_mode(dev, status->mode);
	} else {
		data->common.mode = status->mode;
	}

	return dm_send_control_frame(dev);
}

void dm_rx_data_handler(struct k_work *work)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = (struct dm_motor_data *)(motor_devices[i]->data);
		if (!data->update) {
			continue;
		}
		const struct dm_motor_config *cfg =
			(const struct dm_motor_config *)(motor_devices[i]->config);

		float prev_angle = data->common.angle;
		data->common.angle =
			(uint_to_float(data->RAWangle, -cfg->p_max, cfg->p_max, 16))*RAD2DEG;
		data->common.rpm =
			RADPS2RPM(uint_to_float(data->RAWrpm, -cfg->v_max, cfg->v_max, 12));
		data->common.torque = uint_to_float(data->RAWtorque, -cfg->t_max, cfg->t_max, 12);

		data->delta_deg_sum += data->common.angle - prev_angle;

		data->update = false;
	}
}

void dm_tx_isr_handler(struct k_timer *dummy)
{
	k_work_submit_to_queue(&dm_work_queue, &dm_tx_data_handle);
}

void dm_isr_init_handler(struct k_timer *dummy)
{
	dummy->expiry_fn = dm_tx_isr_handler;
	k_work_submit_to_queue(&dm_work_queue, &dm_init_work);
}

void dm_tx_data_handler(struct k_work *work)
{
	uint64_t now = k_uptime_get();

	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = motor_devices[i]->data;
		const struct dm_motor_config *cfg = motor_devices[i]->config;

		if (data->common.link.requested_enabled &&
		    now - data->last_tx_time >= dm_control_period_ms(cfg)) {
			int ret;

			ret = dm_send_cmd_frame_reply(motor_devices[i], enable_frame,
						      "dm-enable-poll");
			if (ret == 0) {
				data->last_tx_time = now;
				data->tx_cnt++;
				if (motor_link_note_missed_reply(motor_devices[i],
								 &data->common.link,
								 DM_OFFLINE_MISSED_REPLIES)) {
					data->enabled = false;
				}
			} else {
				motor_stats_inc(MOTOR_STAT_TX_ERROR);
			}
			if (data->err > 1 && data->tx_cnt >= 3U) {
				if (dm_send_cmd_frame(motor_devices[i], clear_error_frame,
						      "dm-clear-error") < 0) {
					motor_stats_inc(MOTOR_STAT_TX_ERROR);
				}
				data->tx_cnt = 0;
			}
		}
		k_sleep(K_USEC(130));
	}
}

void dm_init_handler(struct k_work *work)
{
	k_timer_stop(&dm_tx_timer);

	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = motor_devices[i]->data;
		const struct dm_motor_config *cfg =
			(const struct dm_motor_config *)(motor_devices[i]->config);

		motor_can_sched_register_can(cfg->common.phy);

		data->filter.id = cfg->common.rx_id & 0xFF;
		data->filter.mask = 0x7FF;

		int err = can_add_rx_filter(cfg->common.phy, dm_rx_handler,
					    (void *)motor_devices[i], &data->filter);
		if (err < 0) {
			motor_stats_inc(MOTOR_STAT_CAN_FILTER_ERROR);
		}
	}

	k_sleep(K_MSEC(500));

	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct dm_motor_data *data = motor_devices[i]->data;

		data->prev_recv_time = k_uptime_get_32();
	}

	k_timer_start(&dm_tx_timer, K_NO_WAIT, K_MSEC(DM_TX_TIMER_PERIOD_MS));
	k_timer_user_data_set(&dm_tx_timer, &dm_tx_data_handle);
}

DT_INST_FOREACH_STATUS_OKAY(DMMOTOR_INST)
