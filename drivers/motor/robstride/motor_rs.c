#include "motor_mi.h"
#include "motor_rs.h"
#include "zephyr/device.h"
#include "zephyr/drivers/can.h"
#include "../common/common.h"
#include "zephyr/drivers/motor.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/_stdint.h>
#include <zephyr/drivers/pid.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#define DT_DRV_COMPAT rs_motor

LOG_MODULE_REGISTER(motor_rs, CONFIG_MOTOR_LOG_LEVEL);

struct k_sem tx_frame_sem;

static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	uint32_t span = (1 << bits) - 1;
	float offset = x_max - x_min;
	return offset * x / span + x_min;
}
int float_to_uint(float x, float x_min, float x_max, int bits)
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

int rs_init(const struct device *dev)
{
	LOG_DBG("rs_init");
	const struct rs_motor_cfg *cfg = dev->config;
	if (!device_is_ready(cfg->common.phy)) {
		LOG_ERR("CAN device not ready");
		return -1;
	}
	if (k_work_busy_get(&rs_init_work) != 0) {
		return 0;
	}

	struct can_filter filter = {0};
	filter.flags = CAN_FILTER_IDE;
	filter.mask = CAN_FILTER_MASK;

	struct rs_can_id id = {
		.master_id = cfg->common.rx_id,
		.motor_id = cfg->common.tx_id,
		.msg_type = Communication_Type_MotorFeedback,
		.reserved = 0,
	};
	filter.id = *(uint32_t *)&id;
	int err = can_add_rx_filter(cfg->common.phy, rs_can_rx_handler, (void *)dev, &filter);
	if (err < 0) {
		LOG_ERR("Error adding CAN filter (err %d)", err);
		return -1;
	}
	return 0;
}

void rs_motor_control(const struct device *dev, enum motor_cmd cmd)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	struct rs_can_id id = {
		.master_id = cfg->common.rx_id,
		.motor_id = cfg->common.tx_id,
		.reserved = 0,
	};
	struct can_frame frame = {
		.data = {0},
		.dlc = 8,
		.flags = CAN_FRAME_IDE,
	};
	switch (cmd) {
	case ENABLE_MOTOR:
		id.msg_type = Communication_Type_MotorEnable;
		frame.id = *(uint32_t*)&id;
		can_send_queued(cfg->common.phy, &frame);
		data->online = true;
		data->enabled = true;
		break;
	case DISABLE_MOTOR:
		id.msg_type = Communication_Type_MotorStop;
		frame.id = *(uint32_t*)&id;
		can_send_queued(cfg->common.phy, &frame);
		data->online = false;
		data->enabled = false;
		break;
	case SET_ZERO:
		id.msg_type = Communication_Type_SetPosZero;
		frame.id = *(uint32_t*)&id;
		frame.data[0] = 0x01;
		data->common.angle = 0;
		can_send_queued(cfg->common.phy, &frame);
		break;

	case CLEAR_PID:

		break;
	case CLEAR_ERROR:
		id.msg_type = Communication_Type_MotorStop;
		frame.id = *(uint32_t*)&id;
		frame.data[0] = 0x01;
		can_send_queued(cfg->common.phy, &frame);
		data->online = false;
		data->enabled = false;
		break;
	}
}

static void rs_motor_pack(const struct device *dev, struct can_frame *frame)
{
	uint32_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp, cur_tep;

	struct rs_motor_data *data = (struct rs_motor_data *)(dev->data);
	struct rs_motor_cfg *cfg = (struct rs_motor_cfg *)(dev->config);
	struct rs_can_id *rs_can_id = (struct rs_can_id *)&(frame->id);

	rs_can_id->motor_id = cfg->common.tx_id;
	rs_can_id->master_id = cfg->common.rx_id;

	frame->dlc = 8;
	frame->flags = 0;

	struct can_frame *frame_follow = &frame[1];
	frame->flags = CAN_FRAME_IDE;
	frame_follow->flags = CAN_FRAME_IDE;
	struct rs_can_id *rs_can_id_fol = (struct rs_can_id *)&(frame_follow->id);
	uint16_t index[2];
	rs_can_id_fol->motor_id = cfg->common.tx_id;
	rs_can_id_fol->master_id = cfg->common.rx_id;
	switch (data->common.mode) {
	case MIT:
		rs_can_id->msg_type = Communication_Type_MotionControl_MIT;

		pos_tmp = float_to_uint(data->target_pos, P_MIN, P_MAX, 16);
		vel_tmp = float_to_uint(data->target_radps, V_MIN, V_MAX, 16);
		kp_tmp = float_to_uint(data->params.k_p, KP_MIN, KP_MAX, 16);
		kd_tmp = float_to_uint(data->params.k_d, KD_MIN, KD_MAX, 16);
		tor_tmp = float_to_uint(data->target_torque, T_MIN, T_MAX, 16);
		rs_can_id->msg_type = tor_tmp;
		frame->data[0] = (pos_tmp >> 8) & 0xFF;
		frame->data[1] = pos_tmp & 0xFF;
		frame->data[2] = (vel_tmp >> 8) & 0xFF;
		frame->data[3] = vel_tmp & 0xFF;
		frame->data[4] = (kp_tmp >> 8) & 0xFF;
		frame->data[5] = kp_tmp & 0xFF;
		frame->data[6] = (kd_tmp >> 8) & 0xFF;
		frame->data[7] = kd_tmp & 0xFF;
		break;
	case PV:
		rs_can_id->msg_type = Communication_Type_SetSingleParameter;
		index[0] = Limit_Spd;
		index[1] = Loc_Ref;
		memcpy(&frame->data[0], &index[0], 2);

		frame->data[2] = 0;
		frame->data[3] = 0;
		memcpy(&frame->data[4], &data->target_radps, 4);

		rs_can_id_fol->msg_type = Communication_Type_SetSingleParameter;
		frame_follow->dlc = 8;

		memcpy(&frame_follow->data[0], &index[1], 2);
		frame_follow->data[2] = 0;
		frame_follow->data[3] = 0;
		memcpy(&frame_follow->data[4], &data->target_pos, 4);

		break;
	case VO:
		rs_can_id->msg_type = Communication_Type_SetSingleParameter;
		vel_tmp = float_to_uint(data->target_radps, V_MIN, V_MAX, 32);
		cur_tep = float_to_uint(data->limit_cur, 0, CUR_MAX, 32);
		index[0] = Limit_Cur;
		index[1] = Spd_Ref;
		memcpy(&frame->data[0], &index[0], 2);

		frame->data[2] = 0;
		frame->data[3] = 0;
		memcpy(&frame->data[4], &data->limit_cur, 4);

		rs_can_id_fol->msg_type = Communication_Type_SetSingleParameter;

		frame_follow->dlc = 8;

		memcpy(&frame_follow->data[0], &index[1], 2);
		frame_follow->data[2] = 0;
		frame_follow->data[3] = 0;
		memcpy(&frame_follow->data[4], &data->target_radps, 4);

		break;
	default:
		break;
	}
}

int rs_motor_set_mode(const struct device *dev, enum motor_mode mode)
{

	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	char mode_str[10] = {0};

	data->common.mode = mode;

	switch (mode) {
	case MIT:
		strcpy(mode_str, "mit");
		break;
	default:
		LOG_DBG("Unsupported motor mode: %d", mode);
		return -ENOSYS;
	}
	struct can_frame frame = {0};
	struct rs_can_id *rs_can_id = (struct rs_can_id *)&(frame.id);
	rs_can_id->msg_type = Communication_Type_SetSingleParameter;
	rs_can_id->motor_id = cfg->common.tx_id;
	rs_can_id->master_id = cfg->common.rx_id;
	frame.flags = CAN_FRAME_IDE;
	frame.dlc = 8;
	uint16_t index = Run_mode;
	memcpy(&frame.data[0], &index, 2);

	frame.data[4] = (uint8_t)mode;
	can_send_queued(cfg->common.phy, &frame);

	for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.capabilities); i++) {
		if (cfg->common.pid_datas[i]->pid_dev == NULL) {
			LOG_ERR("PID params not found for mode: %d", mode);
			break;
		}
		if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
			struct pid_config params;
			pid_get_params(cfg->common.pid_datas[i], &params);

			data->common.mode = mode;
			data->params.k_p = params.k_p;
			data->params.k_d = params.k_d;
			break;
		}
	}
	return 0;
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

static void rs_can_rx_handler(const struct device *can_dev, struct can_frame *frame,
			      void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	struct rs_motor_data *data = (struct rs_motor_data *)(dev->data);
	struct rs_can_id *can_id = (struct rs_can_id *)&(frame->id);
	if (can_id->msg_type == Communication_Type_MotorFeedback ||
		can_id->msg_type == Communication_Type_MotorReport) {
		data->err = ((frame->data[0]) >> 8) & 0x1f;
		if (data->err) {
			LOG_ERR("Error code: %d on motor %s", data->err, dev->name);
		}
		data->RAWangle = (frame->data[0] << 8) | (frame->data[1]);
		data->RAWrpm = (frame->data[2] << 8) | (frame->data[3]);
		data->RAWtorque = (frame->data[4] << 8) | (frame->data[5]);
		data->RAWtemp = (frame->data[6] << 8) | (frame->data[7]);
	}
}

void rs_rx_data_handler(struct k_work *work)
{
	// LOG_DBG("rs_rx_data_handler");
	for (int i = 0; i < MOTOR_COUNT; i++) {
		struct rs_motor_data *data = (struct rs_motor_data *)(motor_devices[i]->data);
		if (!data->update) {
			continue;
		}

		float prev_angle = data->common.angle;
		data->common.angle =
			(uint16_to_float(data->RAWangle, (double)P_MIN, (double)P_MAX, 16)) *
			RAD2DEG;
		data->common.rpm =
			RADPS2RPM(uint16_to_float(data->RAWrpm, (double)V_MIN, (double)V_MAX, 16));
		data->common.torque =
			uint16_to_float(data->RAWtorque, (double)T_MIN, (double)T_MAX, 16);
		data->common.temperature = ((float)(data->RAWtemp)) / 10;
		data->delta_deg_sum += data->common.angle - prev_angle;

		data->update = false;
	}
}

void rs_tx_isr_handler(struct k_timer *dummy)
{
	// LOG_DBG("rs_tx_isr_handler");
	k_work_submit_to_queue(&rs_work_queue, &rs_tx_data_handle);
}

void rs_isr_init_handler(struct k_timer *dummy)
{
	LOG_DBG("rs_isr_init_handler");
	dummy->expiry_fn = rs_tx_isr_handler;
	k_work_queue_start(&rs_work_queue, rs_work_queue_stack, CAN_SEND_STACK_SIZE,
			   CAN_SEND_PRIORITY, NULL);
	k_work_submit_to_queue(&rs_work_queue, &rs_init_work);
}

int rs_get(const struct device *dev, motor_status_t *status)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;

	data->common.angle = RAD2DEG(uint16_to_float(data->RAWangle, (double)-cfg->p_max, (double)cfg->p_max, 16));
	data->common.rpm = RADPS2RPM(uint16_to_float(data->RAWrpm, (double)-cfg->v_max, (double)cfg->v_max, 16));
	data->common.torque = uint16_to_float(data->RAWtorque, (double)-cfg->t_max, (double)cfg->t_max, 16);
	data->common.temperature = ((float)(data->RAWtemp)) / 10.0f;

	status->angle = fmodf(data->common.angle, 360.0f);
	status->rpm = data->common.rpm;
	status->torque = data->common.torque;
	status->temperature = data->common.temperature;
	status->mode = data->common.mode;
	status->sum_angle = data->common.angle;
	status->speed_limit[0] = cfg->v_max;
	status->speed_limit[1] = -cfg->v_max;
	status->torque_limit[0] = cfg->t_max;
	status->torque_limit[1] = -cfg->t_max;

	return 0;
}

int rs_send_mit(const struct device *dev, motor_status_t *status) {
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	struct rs_can_id id = {
		.msg_type = Communication_Type_MotionControl_MIT,
		.motor_id = cfg->common.tx_id,
		.reserved = 0,
	};
	struct can_frame frame = {
		.data = {0},
		.dlc = 8,
		.flags = CAN_FRAME_IDE,
	};
	uint16_t pos_tmp = float_to_uint(data->target_pos, P_MIN, P_MAX, 16);
	uint16_t vel_tmp = float_to_uint(data->target_radps, V_MIN, V_MAX, 16);
	uint16_t kp_tmp = float_to_uint(data->params.k_p, KP_MIN, KP_MAX, 16);
	uint16_t kd_tmp = float_to_uint(data->params.k_d, KD_MIN, KD_MAX, 16);
	uint16_t tor_tmp = float_to_uint(data->target_torque, T_MIN, T_MAX, 16);
	id.master_id = tor_tmp;
	frame.data[0] = (pos_tmp >> 8) & 0xFF;
	frame.data[1] = pos_tmp & 0xFF;
	frame.data[2] = (vel_tmp >> 8) & 0xFF;
	frame.data[3] = vel_tmp & 0xFF;
	frame.data[4] = (kp_tmp >> 8) & 0xFF;
	frame.data[5] = kp_tmp & 0xFF;
	frame.data[6] = (kd_tmp >> 8) & 0xFF;
	frame.data[7] = kd_tmp & 0xFF;
	can_send_queued(cfg->common.phy, &frame);
}

int rs_send_pid_params(const struct device *dev, motor_status_t *status) {
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	struct rs_can_id id = {
		.msg_type = Communication_Type_SetSingleParameter,
		.motor_id = cfg->common.tx_id,
		.reserved = 0,
	};
}

int rs_set(const struct device *dev, motor_status_t *status)
{
	struct rs_motor_data *data = dev->data;
	const struct rs_motor_cfg *cfg = dev->config;
	char mode_str[10] = {0};
	if (status->mode == MIT) {
		strcpy(mode_str, "mit");
		data->target_pos = status->angle / RAD2DEG;
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_torque = status->torque;
	} else if (status->mode == PV) {
		strcpy(mode_str, "pv");
		data->target_pos = status->angle / RAD2DEG;
		data->target_radps = RPM2RADPS(status->rpm);
	} else if (status->mode == VO) {
		data->target_radps = RPM2RADPS(status->rpm);
		data->target_pos = 0;
		data->target_torque = 0;
	} else {
		return -ENOSYS;
	}
	if (status->mode == MIT) {
		for (int i = 0; i < SIZE_OF_ARRAY(cfg->common.capabilities); i++) {
			if (cfg->common.pid_datas[i]->pid_dev == NULL) {
				break;
			}
			if (strcmp(cfg->common.capabilities[i], mode_str) == 0) {
				struct pid_config params;
				pid_get_params(cfg->common.pid_datas[i], &params);

				data->common.mode = status->mode;
				data->params.k_p = params.k_p;
				data->params.k_d = params.k_d;
				break;
			}
		}
	}
	if (status->mode != data->common.mode) {
		// LOG_DBG("rs_set: mode changed from %d to %d", data->common.mode, status->mode);
		data->common.mode = status->mode;
		if (rs_motor_set_mode(dev, status->mode) < 0) {
			LOG_ERR("Failed to set motor mode");
			return -EIO;
		}
	} else {
		return 0; // No mode change, no need to set
	}
}

DT_INST_FOREACH_STATUS_OKAY(MIMOTOR_INST)
