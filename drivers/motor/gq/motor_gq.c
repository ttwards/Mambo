/*
 * Copyright (c) 2026 ttwards <12411711@mail.sustech.edu.cn>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "motor_gq.h"

#include <errno.h>
#include <math.h>
#include <string.h>

#include "../common/common.h"
#include "../common/motor_can_sched.h"
#include "../common/motor_link.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#define DT_DRV_COMPAT gq_motor

LOG_MODULE_REGISTER(motor_gq, CONFIG_MOTOR_LOG_LEVEL);

static uint32_t gq_motor_id(const struct gq_motor_config *cfg)
{
	return cfg->common.id & 0x7FU;
}

static uint32_t gq_classic_tx_id(const struct gq_motor_config *cfg)
{
	return cfg->common.tx_id != 0 ? (uint32_t)cfg->common.tx_id : gq_motor_id(cfg);
}

static uint32_t gq_reply_id(const struct gq_motor_config *cfg)
{
	return cfg->common.rx_id != 0 ? (uint32_t)cfg->common.rx_id
				      : (gq_motor_id(cfg) << GQ_REPLY_ID_SHIFT);
}

static int16_t gq_clamp_i16(float value)
{
	if (isnan(value)) {
		return GQ_INT16_NAN;
	}
	if (value > (float)INT16_MAX) {
		motor_stats_inc(MOTOR_STAT_LIMIT_CLAMP);
		return INT16_MAX;
	}
	if (value < (float)INT16_MIN) {
		motor_stats_inc(MOTOR_STAT_LIMIT_CLAMP);
		return INT16_MIN;
	}
	return (int16_t)value;
}

static int16_t gq_angle_to_raw(float angle_deg)
{
	return gq_clamp_i16(angle_deg / 360.0f * GQ_POS_SCALE_INT16);
}

static int16_t gq_rpm_to_raw(float rpm)
{
	return gq_clamp_i16(rpm / 60.0f * GQ_VEL_SCALE_INT16);
}

static int16_t gq_torque_to_raw(float torque)
{
	return gq_clamp_i16(torque * GQ_TORQUE_SCALE_INT16);
}

static int16_t gq_pid_to_raw(float gain)
{
	return gq_clamp_i16(gain * GQ_PID_SCALE_INT16);
}

static float gq_raw_pos_to_angle(int16_t raw)
{
	return (float)raw / GQ_POS_SCALE_INT16 * 360.0f;
}

static float gq_raw_vel_to_rpm(int16_t raw)
{
	return (float)raw / GQ_VEL_SCALE_INT16 * 60.0f;
}

static float gq_raw_torque_to_nm(int16_t raw)
{
	return (float)raw / GQ_TORQUE_SCALE_INT16;
}

static void gq_put_i16(uint8_t *dst, int16_t value)
{
	sys_put_le16((uint16_t)value, dst);
}

static int16_t gq_get_i16(const uint8_t *src)
{
	return (int16_t)sys_get_le16(src);
}

static void gq_init_frame(struct can_frame *frame, uint32_t id, uint8_t len, bool extended,
			  bool canfd)
{
	*frame = (struct can_frame){
		.id = id,
		.dlc = can_bytes_to_dlc(len),
		.flags = extended ? CAN_FRAME_IDE : 0,
	};

	if (canfd) {
		frame->flags |= CAN_FRAME_FDF | CAN_FRAME_BRS;
	}
}

static int gq_send_frame(const struct device *dev, const struct can_frame *frame, const char *tag)
{
	const struct gq_motor_config *cfg = dev->config;
	int ret = motor_can_sched_send_with_priority(cfg->common.phy, frame,
						     MOTOR_CAN_SCHED_PRIO_CRITICAL, tag);

	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_TX_ERROR);
	}
	return ret;
}

static void gq_pack_classic_stop(const struct device *dev, struct can_frame *frame, bool brake)
{
	const struct gq_motor_config *cfg = dev->config;

	gq_init_frame(frame, gq_classic_tx_id(cfg), 3, gq_classic_tx_id(cfg) > CAN_STD_ID_MASK,
		      false);
	frame->data[0] = 0x01;
	frame->data[1] = 0x00;
	frame->data[2] = brake ? 0x0F : 0x00;
}

static void gq_pack_classic_zero(const struct device *dev, struct can_frame *frame)
{
	const struct gq_motor_config *cfg = dev->config;
	uint32_t id = GQ_CLASSIC_EXT_PREFIX_CONFIG | gq_motor_id(cfg);
	const uint8_t cmd[] = {0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0A};

	gq_init_frame(frame, id, sizeof(cmd), true, false);
	memcpy(frame->data, cmd, sizeof(cmd));
}

static void gq_pack_classic_control(const struct device *dev, struct can_frame *frame)
{
	const struct gq_motor_config *cfg = dev->config;
	struct gq_motor_data *data = dev->data;
	int16_t pos = gq_angle_to_raw(data->target_angle);
	int16_t vel = gq_rpm_to_raw(data->target_rpm);
	int16_t tqe = gq_torque_to_raw(data->target_torque);

	switch (data->common.mode) {
	case MIT:
		gq_init_frame(frame, GQ_CLASSIC_EXT_PREFIX_MIT | gq_motor_id(cfg), 8, true, false);
		int16_t kp = gq_pid_to_raw(data->kp);
		int16_t kd = gq_pid_to_raw(data->kd);

		frame->data[0] = pos & 0xFF;
		frame->data[1] = (pos >> 8) & 0xFF;
		frame->data[2] = vel & 0xFF;
		frame->data[3] = ((vel >> 8) & 0x0F) | ((tqe & 0x0F) << 4);
		frame->data[4] = (tqe >> 4) & 0xFF;
		frame->data[5] = kp & 0xFF;
		frame->data[6] = ((kp >> 8) & 0x0F) | ((kd & 0x0F) << 4);
		frame->data[7] = kd >> 4;
		break;
	case PV:
		gq_init_frame(frame, gq_classic_tx_id(cfg), 8,
			      gq_classic_tx_id(cfg) > CAN_STD_ID_MASK, false);
		frame->data[0] = 0x07;
		frame->data[1] = 0x35;
		gq_put_i16(&frame->data[2], vel);
		gq_put_i16(&frame->data[4], data->target_torque == 0.0f ? GQ_INT16_NAN : tqe);
		gq_put_i16(&frame->data[6], pos);
		break;
	case VO:
		gq_init_frame(frame, gq_classic_tx_id(cfg),
			      data->common.target == MOTOR_TARGET_TORQUE ? 4 : 8,
			      gq_classic_tx_id(cfg) > CAN_STD_ID_MASK, false);
		if (data->common.target == MOTOR_TARGET_TORQUE) {
			frame->data[0] = 0x05;
			frame->data[1] = 0x13;
			gq_put_i16(&frame->data[2], tqe);
		} else {
			frame->data[0] = 0x07;
			frame->data[1] = 0x07;
			gq_put_i16(&frame->data[2], GQ_INT16_NAN);
			gq_put_i16(&frame->data[4], vel);
			gq_put_i16(&frame->data[6], GQ_INT16_NAN);
		}
		break;
	default:
		break;
	}
}

#ifdef CONFIG_CAN_FD_MODE
static void gq_pack_fd_stop(const struct device *dev, struct can_frame *frame, bool brake)
{
	const struct gq_motor_config *cfg = dev->config;
	uint32_t id = GQ_CLASSIC_EXT_PREFIX_CONFIG | gq_motor_id(cfg);
	const uint8_t cmd_stop[] = {0x01, 0x00, 0x00, 0x05, 0x06, 0x0F, 0x00};
	const uint8_t cmd_brake[] = {0x01, 0x00, 0x0F, 0x05, 0x06, 0x0F, 0x00};
	const uint8_t *cmd = brake ? cmd_brake : cmd_stop;

	gq_init_frame(frame, id, 7, true, true);
	memcpy(frame->data, cmd, 7);
}

static void gq_pack_fd_zero(const struct device *dev, struct can_frame *frame)
{
	const struct gq_motor_config *cfg = dev->config;
	uint32_t id = GQ_CLASSIC_EXT_PREFIX_CONFIG | gq_motor_id(cfg);
	const uint8_t cmd[] = {0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0A};

	gq_init_frame(frame, id, sizeof(cmd), true, true);
	memcpy(frame->data, cmd, sizeof(cmd));
}

static void gq_pack_fd_control(const struct device *dev, struct can_frame *frame)
{
	const struct gq_motor_config *cfg = dev->config;
	struct gq_motor_data *data = dev->data;
	uint32_t id = GQ_CLASSIC_EXT_PREFIX_CONFIG | gq_motor_id(cfg);
	int16_t pos = gq_angle_to_raw(data->target_angle);
	int16_t vel = gq_rpm_to_raw(data->target_rpm);
	int16_t tqe = gq_torque_to_raw(data->target_torque);

	switch (data->common.mode) {
	case MIT:
		gq_init_frame(frame, id, 24, true, true);
		memcpy(frame->data, (uint8_t[]){0x01, 0x00, 0x0A, 0x07, 0x20, 0x00, 0x00, 0x00,
						0x00, 0x00, 0x00, 0x06, 0x2B, 0x00, 0x00, 0x00,
						0x00, 0x14, 0x04, 0x00, 0x11, 0x0F, 0x50, 0x50},
		       24);
		gq_put_i16(&frame->data[5], pos);
		gq_put_i16(&frame->data[7], vel);
		gq_put_i16(&frame->data[9], tqe);
		gq_put_i16(&frame->data[13], gq_pid_to_raw(data->kp));
		gq_put_i16(&frame->data[15], gq_pid_to_raw(data->kd));
		break;
	case PV:
		gq_init_frame(frame, id, 20, true, true);
		memcpy(frame->data,
		       (uint8_t[]){0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80, 0x00, 0x00, 0x06,
				   0x25, 0x00, 0x00, 0x00, 0x00, 0x14, 0x04, 0x00, 0x11, 0x0F},
		       20);
		gq_put_i16(&frame->data[7], vel);
		gq_put_i16(&frame->data[11], data->target_torque == 0.0f ? GQ_INT16_NAN : tqe);
		gq_put_i16(&frame->data[13], pos);
		break;
	case VO:
		if (data->common.target == MOTOR_TARGET_TORQUE) {
			gq_init_frame(frame, id, 24, true, true);
			memcpy(frame->data,
			       (uint8_t[]){0x01, 0x00, 0x0A, 0x04, 0x06, 0x20, 0x00, 0x80,
					   0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00,
					   0x00, 0x80, 0x14, 0x04, 0x00, 0x11, 0x0F, 0x50},
			       24);
			gq_put_i16(&frame->data[10], tqe);
		} else {
			gq_init_frame(frame, id, 20, true, true);
			memcpy(frame->data, (uint8_t[]){0x01, 0x00, 0x0A, 0x06, 0x20, 0x00, 0x80,
							0x00, 0x00, 0x14, 0x04, 0x00, 0x11, 0x0F,
							0x50, 0x50, 0x00, 0x00, 0x00, 0x00},
			       20);
			gq_put_i16(&frame->data[7], vel);
		}
		break;
	default:
		break;
	}
}
#endif /* CONFIG_CAN_FD_MODE */

static void gq_parse_state_int16(const struct device *dev, uint8_t mode, uint8_t fault, int16_t pos,
				 int16_t vel, int16_t tqe)
{
	struct gq_motor_data *data = dev->data;

	ARG_UNUSED(mode);
	data->common.angle = gq_raw_pos_to_angle(pos);
	data->common.rpm = gq_raw_vel_to_rpm(vel);
	data->common.torque = gq_raw_torque_to_nm(tqe);
	data->error = fault;
	if (fault != 0) {
		motor_stats_inc(MOTOR_STAT_DRIVER_ERROR);
	}
	motor_link_observe_reply(dev, &data->common.link);
}

static void gq_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data)
{
	const struct device *dev = user_data;
	struct gq_motor_data *data = dev->data;
	uint8_t len = can_dlc_to_bytes(frame->dlc);

	motor_can_sched_report_rx(can_dev, frame);

	if ((frame->flags & CAN_FRAME_IDE) == 0 || len < 5) {
		motor_stats_inc(MOTOR_STAT_UNKNOWN_RX);
		return;
	}

	uint8_t id = (frame->id >> GQ_REPLY_ID_SHIFT) & 0xFFU;
	const struct gq_motor_config *cfg = dev->config;

	if (id != gq_motor_id(cfg)) {
		motor_stats_inc(MOTOR_STAT_UNKNOWN_RX);
		return;
	}

	if (len == 8 && frame->data[0] == 0x27 && frame->data[1] == 0x01) {
		gq_parse_state_int16(dev, data->common.mode, 0, gq_get_i16(&frame->data[2]),
				     gq_get_i16(&frame->data[4]), gq_get_i16(&frame->data[6]));
		return;
	}
	if (len == 8 && frame->data[0] != 0x27) {
		gq_parse_state_int16(dev, frame->data[0], frame->data[1],
				     gq_get_i16(&frame->data[2]), gq_get_i16(&frame->data[4]),
				     gq_get_i16(&frame->data[6]));
		return;
	}
#ifdef CONFIG_CAN_FD_MODE
	if (len >= 14 && frame->data[0] == 0x24 && frame->data[1] == 0x04 &&
	    frame->data[2] == 0x00 && frame->data[11] == 0x21 && frame->data[12] == 0x0F) {
		gq_parse_state_int16(dev, frame->data[3], frame->data[13],
				     gq_get_i16(&frame->data[5]), gq_get_i16(&frame->data[7]),
				     gq_get_i16(&frame->data[9]));
		return;
	}
#endif
	if (len == 7 && frame->data[0] == 0x41 && frame->data[1] == 0x01 &&
	    frame->data[2] == 0x04 && frame->data[3] == 0x4F && frame->data[4] == 0x4B &&
	    frame->data[5] == 0x0D && frame->data[6] == 0x0A) {
		motor_link_observe_reply(dev, &data->common.link);
		return;
	}

	motor_stats_inc(MOTOR_STAT_UNKNOWN_RX);
}

static int gq_check_canfd_support(const struct device *dev)
{
	const struct gq_motor_config *cfg = dev->config;

	if (!cfg->enable_canfd) {
		return 0;
	}

#ifndef CONFIG_CAN_FD_MODE
	return -ENOTSUP;
#else
	can_mode_t capabilities = 0;
	int ret = can_get_capabilities(cfg->common.phy, &capabilities);

	if (ret < 0) {
		return ret;
	}
	if ((capabilities & CAN_MODE_FD) == 0) {
		LOG_ERR("%s does not support CAN FD", cfg->common.phy->name);
		return -ENOTSUP;
	}

	can_mode_t mode = can_get_mode(cfg->common.phy);

	if ((mode & CAN_MODE_FD) != 0) {
		return 0;
	}

	ret = can_set_mode(cfg->common.phy, mode | CAN_MODE_FD);
	if (ret == 0) {
		return 0;
	}
	if (ret == -EBUSY && (can_get_mode(cfg->common.phy) & CAN_MODE_FD) != 0) {
		return 0;
	}

	LOG_ERR("failed to enable CAN FD mode on %s: %d", cfg->common.phy->name, ret);
	return ret;
#endif
}

int gq_init(const struct device *dev)
{
	const struct gq_motor_config *cfg = dev->config;
	struct gq_motor_data *data = dev->data;
	int ret;

	if (!device_is_ready(cfg->common.phy)) {
		motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
		return -ENODEV;
	}

	ret = gq_check_canfd_support(dev);
	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
		return ret;
	}

	ret = motor_can_sched_register_can(cfg->common.phy);
	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
		return ret;
	}

	data->filter = (struct can_filter){
		.id = gq_reply_id(cfg),
		.mask = GQ_REPLY_ID_MASK,
		.flags = CAN_FILTER_IDE,
	};
	data->filter_id =
		can_add_rx_filter(cfg->common.phy, gq_can_rx_handler, (void *)dev, &data->filter);
	if (data->filter_id < 0) {
		motor_stats_inc(MOTOR_STAT_CAN_FILTER_ERROR);
		return data->filter_id;
	}

	return 0;
}

void gq_control(const struct device *dev, enum motor_cmd cmd)
{
	const struct gq_motor_config *cfg = dev->config;
	struct gq_motor_data *data = dev->data;
	struct can_frame frame = {0};

	switch (cmd) {
	case ENABLE_MOTOR:
		motor_link_request_enable(&data->common.link);
		break;
	case DISABLE_MOTOR:
		if (cfg->enable_canfd) {
#ifdef CONFIG_CAN_FD_MODE
			gq_pack_fd_stop(dev, &frame, false);
#else
			return;
#endif
		} else {
			gq_pack_classic_stop(dev, &frame, false);
		}
		(void)gq_send_frame(dev, &frame, "gq-disable");
		motor_link_request_disable(&data->common.link);
		break;
	case SET_ZERO:
		if (cfg->enable_canfd) {
#ifdef CONFIG_CAN_FD_MODE
			gq_pack_fd_zero(dev, &frame);
#else
			return;
#endif
		} else {
			gq_pack_classic_zero(dev, &frame);
		}
		(void)gq_send_frame(dev, &frame, "gq-set-zero");
		break;
	case CLEAR_ERROR:
		if (cfg->enable_canfd) {
#ifdef CONFIG_CAN_FD_MODE
			gq_pack_fd_stop(dev, &frame, true);
#else
			return;
#endif
		} else {
			gq_pack_classic_stop(dev, &frame, true);
		}
		(void)gq_send_frame(dev, &frame, "gq-brake");
		break;
	case CLEAR_CONTROLLER:
		data->kp = 0.0f;
		data->kd = 0.0f;
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_CMD);
		break;
	}
}

int gq_set(const struct device *dev, motor_setpoint_t *setpoint)
{
	const struct gq_motor_config *cfg = dev->config;
	struct gq_motor_data *data = dev->data;
	motor_controller_info_t controller = {0};
	struct can_frame frame = {0};
	int ret;

	ret = motor_resolve_controller(dev, setpoint, &controller);
	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		return ret;
	}
	if (setpoint->target == MOTOR_TARGET_NONE) {
		return 0;
	}

	data->common.mode = setpoint->mode;
	data->common.target = setpoint->target;
	data->common.controller_id = setpoint->controller_id;
	data->target_angle = setpoint->angle;
	data->target_rpm = setpoint->rpm;
	data->target_torque = setpoint->torque;
	data->kp = 0.0f;
	data->kd = 0.0f;

	if (cfg->common.controllers[setpoint->controller_id].param_count > 0) {
		struct motor_controller_params params = {0};

		if (motor_controller_get_params(&cfg->common.controllers[setpoint->controller_id],
						0, &params) == 0) {
			data->kp = params.k_p;
			data->kd = params.k_d;
		}
	}

	if (cfg->enable_canfd) {
#ifdef CONFIG_CAN_FD_MODE
		gq_pack_fd_control(dev, &frame);
#else
		return -ENOTSUP;
#endif
	} else {
		gq_pack_classic_control(dev, &frame);
	}

	return gq_send_frame(dev, &frame, "gq-control");
}

int gq_get(const struct device *dev, motor_status_t *status)
{
	struct gq_motor_data *data = dev->data;

	status->online = data->common.link.online;
	status->enabled = data->common.link.requested_enabled;
	status->error = data->error;
	status->angle = data->common.angle;
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;
	status->temperature = data->common.temperature;
	status->sum_angle = data->common.angle;
	status->mode = data->common.mode;
	status->target = data->common.target;
	status->controller_id = data->common.controller_id;
	status->speed_limit[0] = data->common.speed_limit[0];
	status->speed_limit[1] = data->common.speed_limit[1];
	status->torque_limit[0] = data->common.torque_limit[0];
	status->torque_limit[1] = data->common.torque_limit[1];

	return data->common.link.online ? 0 : -ENODEV;
}

DT_INST_FOREACH_STATUS_OKAY(GQ_MOTOR_INST)
