#ifndef SBUS_H_
#define SBUS_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sbus.h>

// SBUS 常量定义
#define SBUS_MIN 353
#define SBUS_MAX 1695

struct sbus_driver_config {
	uint8_t frameSize;
	uint8_t inputChannels;
	uint8_t startByte;
	uint8_t endByte;
};

struct sbus_driver_data {
	uint8_t data[25];
	uint16_t channels[16];
	bool frameLost;
	bool failSafe;
	bool digitalChannels[2];
	int recv_time;
};

#endif // SBUS_H
