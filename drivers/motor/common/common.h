#ifndef MOTOR_COMMON_H_
#define MOTOR_COMMON_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/motor.h>

struct tx_frame {
	const struct device *can_dev;
	struct k_sem *sem;
	struct can_frame frame;
};

int8_t get_can_id(const struct device *dev);
int8_t reg_can_dev(const struct device *dev);

int can_send_queued(const struct device *can_dev, struct can_frame *frame);

// Macros
#define HIGH_BYTE(x)           ((x) >> 8)
#define LOW_BYTE(x)            ((x) & 0xFF)
#define COMBINE_HL8(HIGH, LOW) ((HIGH << 8) + LOW)

#endif // MOTOR_COMMON_H_