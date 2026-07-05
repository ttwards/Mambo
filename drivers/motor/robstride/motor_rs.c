#include "motor_rs.h"
#include "zephyr/device.h"
#include "zephyr/drivers/can.h"
#include "../common/common.h"
#include "../common/motor_link.h"
#include "../common/motor_can_sched.h"
#include "zephyr/drivers/motor.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT rs_motor

LOG_MODULE_REGISTER(motor_rs, CONFIG_MOTOR_LOG_LEVEL);

#define RS_RUN_MODE_MIT               0U
#define RS_RUN_MODE_CSP               5U
#define RS_CSP_DEFAULT_LOC_KP         2.0f
#define RS_CSP_DEFAULT_LIMIT_SPD      0.8f
#define RS_CSP_DEFAULT_LIMIT_CUR      2.0f
#define RS_CSP_DEFAULT_SPD_KI         0.02f
#define RS_FEEDBACK_MASK              0x1F00FF00U
#define RS_OFFLINE_RECOVERY_PERIOD_MS 200U

static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (span * (float)x / (float)((1 << bits) - 1)) + offset;
}
static int float_to_uint(float x, float x_min, float x_max, int bits)
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

static void rs_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data);
static void rs_offline_recovery_handler(struct k_work *work);

static struct can_filter filters[MOTOR_COUNT];
K_WORK_DELAYABLE_DEFINE(rs_offline_recovery_work, rs_offline_recovery_handler);

static uint32_t rs_pack_ext_id(uint8_t msg_type, uint8_t master_id, uint8_t motor_id,
			       uint8_t reserved)
{
	return (((uint32_t)msg_type & 0x1FU) << 24) | ((uint32_t)reserved << 16) |
	       ((uint32_t)master_id << 8) | (uint32_t)motor_id;
}

static uint32_t rs_feedback_id(const struct rs_motor_cfg *cfg)
{
	return ((uint32_t)Communication_Type_MotorFeedback << 24) |
	       (((uint32_t)cfg->common.tx_id & 0xFFU) << 8);
}

static void rs_init_ext_frame(const struct rs_motor_cfg *cfg, struct can_frame *frame,
			      uint8_t msg_type, uint8_t reserved)
{
	*frame = (struct can_frame){
		.id = rs_pack_ext_id(msg_type, cfg->common.rx_id & 0xFF, cfg->common.tx_id & 0xFF,
				     reserved),
		.dlc = 8,
		.flags = CAN_FRAME_IDE,
	};
}

static int rs_send_ext_reply(const struct device *dev, struct can_frame *frame, const char *tag)
{
	const struct rs_motor_cfg *cfg = dev->config;
	int ret = motor_can_sched_send_reply(cfg->common.phy, frame, rs_feedback_id(cfg),
					     RS_FEEDBACK_MASK, 5U, tag);

	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_TX_ERROR);
	}
	return ret;
}

static int rs_send_stop_frame(const struct device *dev, const char *tag)
{
	const struct rs_motor_cfg *cfg = dev->config;
	struct can_frame frame;

	rs_init_ext_frame(cfg, &frame, Communication_Type_MotorStop, 0);
	return rs_send_ext_reply(dev, &frame, tag);
}

static int rs_write_param_float(const struct device *dev, uint16_t index, float value,
				const char *tag)
{
	const struct rs_motor_cfg *cfg = dev->config;
	struct can_frame frame;

	rs_init_ext_frame(cfg, &frame, Communication_Type_SetSingleParameter, 0);
	memcpy(&frame.data[0], &index, sizeof(index));
	memcpy(&frame.data[4], &value, sizeof(value));
	return rs_send_ext_reply(dev, &frame, tag);
}

static int rs_write_param_u8(const struct device *dev, uint16_t index, uint8_t value,
			     const char *tag)
{
	const struct rs_motor_cfg *cfg = dev->config;
	struct can_frame frame;

	rs_init_ext_frame(cfg, &frame, Communication_Type_SetSingleParameter, 0);
	memcpy(&frame.data[0], &index, sizeof(index));
	frame.data[4] = value;
	return rs_send_ext_reply(dev, &frame, tag);
}

static float rs_positive_or(float value, float fallback)
{
	return value > 0.0f ? value : fallback;
}

static float rs_clamp_positive(float value, float fallback, float max)
{
	float out = rs_positive_or(value, fallback);

	if (max > 0.0f && out > max) {
		return max;
	}
	return out;
}

static int get_motor_id(struct can_frame *frame)
{
	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct device *dev = motor_devices[i];
		const struct rs_motor_cfg *cfg = (const struct rs_motor_cfg *)(dev->config);
		if ((cfg->common.tx_id & 0xFF) == (frame->id & 0xFF00) >> 8) {
			return i;
		}
	}
	return -1;
}

static void rs_send_enable_frame(const struct device *dev, const char *tag_prefix)
{
	const struct rs_motor_cfg *cfg = dev->config;
	struct can_frame frame = {
		.data = {0},
		.dlc = 8,
		.flags = CAN_FRAME_IDE,
	};

	frame.id = rs_pack_ext_id(Communication_Type_MotorStop, cfg->common.rx_id & 0xFF,
				  cfg->common.tx_id & 0xFF, 0);
	frame.data[0] = 0x01;
	motor_can_sched_send_prio(cfg->common.phy, &frame, true, tag_prefix);

	frame.id = rs_pack_ext_id(Communication_Type_MotorEnable, cfg->common.rx_id & 0xFF,
				  cfg->common.tx_id & 0xFF, 0);
	frame.data[0] = 0x0;
	motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-enable");
}

static void rs_send_auto_report_frame(const struct device *dev, const char *tag)
{
	const struct rs_motor_cfg *cfg = dev->config;
	struct can_frame frame = {
		.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01, 0x00},
		.dlc = 8,
		.flags = CAN_FRAME_IDE,
	};

	frame.id = rs_pack_ext_id(Communication_Type_MotorReport, cfg->common.rx_id & 0xFF,
				  cfg->common.tx_id & 0xFF, 0);
	motor_can_sched_send_prio(cfg->common.phy, &frame, true, tag);
}

int rs_init(const struct device *dev)
{
	const struct rs_motor_cfg *cfg = dev->config;
	if (!device_is_ready(cfg->common.phy)) {
		motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
		return -1;
	}

	static bool initialized = false;
	if (initialized) {
		return 0;
	}
	initialized = true;

	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct rs_motor_cfg *cfg =
			(const struct rs_motor_cfg *)(motor_devices[i]->config);

		motor_can_sched_register_can(cfg->common.phy);
		filters[i].flags = CAN_FILTER_IDE;
		filters[i].mask = CAN_FILTER_MASK;
		filters[i].id = (cfg->common.tx_id & 0xFF) << 8;
		int err = can_add_rx_filter(cfg->common.phy, rs_can_rx_handler, 0, &filters[i]);
		if (err < 0) {
			motor_stats_inc(MOTOR_STAT_CAN_FILTER_ERROR);
			return -1;
		}
	}
	k_sleep(K_MSEC(100));

	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct rs_motor_cfg *cfg =
			(const struct rs_motor_cfg *)(motor_devices[i]->config);

		if (cfg->auto_report) {
			rs_send_auto_report_frame(motor_devices[i], "rs-report");
			k_sleep(K_MSEC(1));
		}
	}

	k_work_schedule(&rs_offline_recovery_work, K_MSEC(RS_OFFLINE_RECOVERY_PERIOD_MS));
	return 0;
}

void rs_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	struct can_frame frame = {
		.data = {0},
		.dlc = 8,
		.flags = CAN_FRAME_IDE,
	};
	switch (cmd) {
	case ENABLE_MOTOR:
		motor_link_request_enable(&data->common.link);
		rs_send_enable_frame(dev, "rs-stop-before-enable");
		break;
	case DISABLE_MOTOR:
		frame.id = rs_pack_ext_id(Communication_Type_MotorStop, cfg->common.rx_id & 0xFF,
					  cfg->common.tx_id & 0xFF, 0);
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-disable");
		motor_link_request_disable(&data->common.link);
		break;
	case SET_ZERO:
		frame.id = rs_pack_ext_id(Communication_Type_SetPosZero, cfg->common.rx_id & 0xFF,
					  cfg->common.tx_id & 0xFF, 0);
		frame.data[0] = 0x01;
		data->common.angle = 0;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-set-zero");
		break;
	case CLEAR_ERROR:
		frame.id = rs_pack_ext_id(Communication_Type_MotorStop, cfg->common.rx_id & 0xFF,
					  cfg->common.tx_id & 0xFF, 0);
		frame.data[0] = 0x01;
		motor_can_sched_send_prio(cfg->common.phy, &frame, true, "rs-clear-error");
		motor_link_request_disable(&data->common.link);
		break;
	case CLEAR_CONTROLLER:
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_CMD);
		break;
	}
}

static void rs_motor_pack(const struct device *dev, struct can_frame *frame)
{
	uint32_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

	struct rs_motor_data *data = (struct rs_motor_data *)(dev->data);
	struct rs_motor_cfg *cfg = (struct rs_motor_cfg *)(dev->config);
	struct rs_can_id *rs_can_id = (struct rs_can_id *)&(frame->id);

	rs_can_id->motor_id = cfg->common.tx_id;

	frame->dlc = 8;
	frame->flags = CAN_FRAME_IDE;
	switch (data->common.mode) {
	case MIT:
		rs_can_id->msg_type = Communication_Type_MotionControl_MIT;

		pos_tmp = float_to_uint(data->target_pos, -cfg->p_max, cfg->p_max, 16);
		vel_tmp = float_to_uint(data->target_radps, -cfg->v_max, cfg->v_max, 16);
		kp_tmp = float_to_uint(data->params.k_p, KP_MIN, cfg->kp_max, 16);
		kd_tmp = float_to_uint(data->params.k_d, KD_MIN, cfg->kd_max, 16);
		tor_tmp = float_to_uint(data->target_torque, -cfg->t_max, cfg->t_max, 16);
		rs_can_id->master_id = tor_tmp & 0xFF;
		rs_can_id->reserved = (tor_tmp >> 8) & 0xFF;
		frame->data[0] = (pos_tmp >> 8) & 0xFF;
		frame->data[1] = pos_tmp & 0xFF;
		frame->data[2] = (vel_tmp >> 8) & 0xFF;
		frame->data[3] = vel_tmp & 0xFF;
		frame->data[4] = (kp_tmp >> 8) & 0xFF;
		frame->data[5] = kp_tmp & 0xFF;
		frame->data[6] = (kd_tmp >> 8) & 0xFF;
		frame->data[7] = kd_tmp & 0xFF;
		break;
	default:
		break;
	}
	return;
}

static int rs_send_control_frame(const struct device *dev)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	struct can_frame tx_frame = {0};
	int ret;

	if (data->common.mode == PV) {
		float limit_spd = rs_clamp_positive(fabsf(data->target_radps),
						    RS_CSP_DEFAULT_LIMIT_SPD, cfg->v_max);

		ret = rs_write_param_float(dev, Loc_Ref, data->target_pos, "rs-csp-loc-ref");
		if (ret < 0) {
			return ret;
		}
		return rs_write_param_float(dev, Limit_Spd, limit_spd, "rs-csp-limit-spd");
	}

	rs_motor_pack(dev, &tx_frame);
	ret = motor_can_sched_send_prio(cfg->common.phy, &tx_frame, true, "rs-control");
	if (ret < 0) {
		motor_stats_inc(MOTOR_STAT_TX_ERROR);
	}
	return ret;
}

static int rs_apply_controller_mode(const struct device *dev, enum motor_mode mode)
{

	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	char mode_str[10] = {0};
	struct motor_controller_params pos_params = {0};
	struct motor_controller_params vel_params = {0};
	bool found = false;

	switch (mode) {
	case MIT:
		strcpy(mode_str, "mit");
		break;
	case PV:
		strcpy(mode_str, "pv");
		break;
	default:
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		return -ENOSYS;
	}

	for (int i = 0; i < motor_get_controller_count(dev); i++) {
		const struct motor_controller_config *ctrl_cfg = &cfg->common.controllers[i];

		if (ctrl_cfg->param_count == 0) {
			motor_stats_inc(MOTOR_STAT_CONFIG_ERROR);
			break;
		}
		if (data->common.controller_id != MOTOR_CONTROLLER_ID_AUTO &&
		    data->common.controller_id != i) {
			continue;
		}
		if (strcmp(ctrl_cfg->info.name, mode_str) == 0 || ctrl_cfg->info.mode == mode) {
			if (motor_controller_get_params(ctrl_cfg, 0, &pos_params) < 0) {
				continue;
			}
			if (mode == PV &&
			    motor_controller_get_params(ctrl_cfg, 1, &vel_params) < 0) {
				continue;
			}

			data->common.mode = mode;
			data->common.target = MOTOR_TARGET_POSITION;
			data->common.controller_id = i;
			data->params.k_p = pos_params.k_p;
			data->params.k_d = pos_params.k_d;
			found = true;
			break;
		}
	}
	if (!found) {
		motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
		return -ENOSYS;
	}

	if (rs_send_stop_frame(dev, "rs-stop-before-mode") < 0) {
		return -EIO;
	}

	if (mode == MIT) {
		return rs_write_param_u8(dev, Run_mode, RS_RUN_MODE_MIT, "rs-set-mit-mode");
	}

	float loc_kp = rs_positive_or(pos_params.k_p, RS_CSP_DEFAULT_LOC_KP);
	float limit_cur =
		rs_clamp_positive(vel_params.output_limit, RS_CSP_DEFAULT_LIMIT_CUR, cfg->t_max);
	float spd_ki = rs_positive_or(vel_params.k_i, RS_CSP_DEFAULT_SPD_KI);

	if (rs_write_param_u8(dev, Run_mode, RS_RUN_MODE_CSP, "rs-set-csp-mode") < 0) {
		return -EIO;
	}
	if (rs_write_param_float(dev, Loc_Kp, loc_kp, "rs-csp-loc-kp") < 0) {
		return -EIO;
	}
	if (rs_write_param_float(dev, Limit_Cur, limit_cur, "rs-csp-limit-cur") < 0) {
		return -EIO;
	}
	if (rs_write_param_float(dev, Spd_Ki, spd_ki, "rs-csp-spd-ki") < 0) {
		return -EIO;
	}
	return 0;
}

static void rs_offline_recovery_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	for (int i = 0; i < MOTOR_COUNT; i++) {
		const struct device *dev = motor_devices[i];
		struct rs_motor_data *data = dev->data;
		const struct rs_motor_cfg *cfg = dev->config;

		if (data->common.link.requested_enabled && !data->common.link.online) {
			rs_send_enable_frame(dev, "rs-offline-enable");
			if (cfg->auto_report) {
				rs_send_auto_report_frame(dev, "rs-offline-report");
			}
		}
	}

	k_work_schedule(&rs_offline_recovery_work, K_MSEC(RS_OFFLINE_RECOVERY_PERIOD_MS));
}

static void rs_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data)
{
	motor_can_sched_report_rx(can_dev, frame);
	uint32_t id = get_motor_id(frame);
	if (id == -1) {
		motor_stats_inc(MOTOR_STAT_UNKNOWN_RX);
		return;
	}

	const struct device *dev = motor_devices[id];
	struct rs_motor_data *data = (struct rs_motor_data *)(dev->data);
	const struct rs_motor_cfg *cfg = dev->config;
	struct rs_can_id *can_id = (struct rs_can_id *)&(frame->id);

	if (can_id->msg_type == Communication_Type_MotorFeedback ||
	    can_id->msg_type == Communication_Type_MotorReport) {
		data->err = (can_id->reserved) & 0x3f;
		data->mode_state = (can_id->reserved >> 6) & 0x03U;
		if (data->err) {
			motor_stats_inc(MOTOR_STAT_DRIVER_ERROR);
		}
		data->RAWangle = (frame->data[0] << 8) | (frame->data[1]);
		data->RAWrpm = (frame->data[2] << 8) | (frame->data[3]);
		data->RAWtorque = (frame->data[4] << 8) | (frame->data[5]);
		data->RAWtemp = (frame->data[6] << 8) | (frame->data[7]);
		if (!data->common.link.online) {
			data->target_pos =
				uint16_to_float(data->RAWangle, -cfg->p_max, cfg->p_max, 16);
		}
		motor_link_observe_reply(dev, &data->common.link);
	}
}

int rs_get(const struct device *dev, motor_status_t *status)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;

	status->online = data->common.link.online;
	status->enabled = data->common.link.online && data->mode_state == RS_MODE_STATE_MOTOR;
	status->error = data->err;

	if (!data->common.link.online) {
		return -ENODEV;
	}

	data->common.angle = RAD2DEG * uint16_to_float(data->RAWangle, -cfg->p_max, cfg->p_max, 16);
	data->common.rpm = RADPS2RPM(uint16_to_float(data->RAWrpm, -cfg->v_max, cfg->v_max, 16));
	data->common.torque = uint16_to_float(data->RAWtorque, -cfg->t_max, cfg->t_max, 16);
	data->common.temperature = ((float)(data->RAWtemp)) / 10.0f;

	status->angle = data->common.angle;
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;
	status->temperature = data->common.temperature;
	status->mode = data->common.mode;
	status->target = data->common.target;
	status->controller_id = data->common.controller_id;
	status->sum_angle = data->common.angle;
	status->speed_limit[0] = cfg->v_max;
	status->speed_limit[1] = -cfg->v_max;
	status->torque_limit[0] = cfg->t_max;
	status->torque_limit[1] = -cfg->t_max;

	return 0;
}

/**
 * @brief Set motor status: angle, speed, torque, mode
 * @param dev Pointer to the motor device structure: robstride motor device
 * @param status Pointer to the motor status structure
 */
int rs_set(const struct device *dev, motor_setpoint_t *status)
{
	struct rs_motor_data *data = dev->data;
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
		data->target_pos = status->angle / RAD2DEG;
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_torque = status->torque;
	} else if (status->mode == PV) {
		if (status->target != MOTOR_TARGET_NONE &&
		    status->target != MOTOR_TARGET_POSITION) {
			return -ENOSYS;
		}
		data->common.target = MOTOR_TARGET_POSITION;
		data->target_pos = status->angle / RAD2DEG;
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_torque = 0.0f;
	} else {
		return -ENOSYS;
	}
	if (status->mode != previous_mode || status->controller_id != previous_controller_id) {
		if (rs_apply_controller_mode(dev, status->mode) < 0) {
			motor_stats_inc(MOTOR_STAT_UNSUPPORTED_MODE);
			return -EIO;
		}
	} else {
		data->common.mode = status->mode;
	}
	return rs_send_control_frame(dev);
}

DT_INST_FOREACH_STATUS_OKAY(RS_MOTOR_INST)
