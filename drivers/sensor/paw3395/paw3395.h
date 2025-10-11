/**
 * @file paw3395.h
 * @brief Header file for PAW3395 optical mouse sensor driver
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_PAW3395_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_PAW3395_H_

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

/* 寄存器地址定义 */
// 基本信息寄存器
#define PAW3395_REG_PRODUCT_ID          0x00   // 产品ID，RO，Default：0x51
#define PAW3395_REG_REVISION_ID         0x01   // 修订版本ID，RO，Default：0x00
#define PAW3395_REG_MOTION              0x02   // 运动状态，R/W，Default：0x00
#define PAW3395_REG_DELTA_X_L           0x03   // X轴低位增量，RO，Default：0x00
#define PAW3395_REG_DELTA_X_H           0x04   // X轴高位增量，RO，Default：0x00
#define PAW3395_REG_DELTA_Y_L           0x05   // Y轴低位增量，RO，Default：0x00
#define PAW3395_REG_DELTA_Y_H           0x06   // Y轴高位增量，RO，Default：0x00
#define PAW3395_REG_SQUAL               0x07   // 表面质量指标，RO，Default：0x00
#define PAW3395_REG_RAWDATA_SUM         0x08   // 原始数据和，RO，Default：0x00
#define PAW3395_REG_MAXIMUM_RAWDATA     0x09   // 最大原始数据，RO，Default：0x00
#define PAW3395_REG_MINIMUM_RAWDATA     0x0A   // 最小原始数据，RO，Default：0x00
#define PAW3395_REG_SHUTTER_LOWER       0x0B   // 快门下限，RO，Default：0x00
#define PAW3395_REG_SHUTTER_UPPER       0x0C   // 快门上限，RO，Default：0x01
#define PAW3395_REG_OBSERVATION         0x15   // 观测值，R/W，Default：0x80
#define PAW3395_REG_MOTION_BURST        0x16   // 运动突发，R/W，Default：0x00
#define PAW3395_REG_POWER_UP_RESET      0x3A   // 上电复位，WO，No default
#define PAW3395_REG_SHUTDOWN            0x3B   // 关闭，WO，No default
#define PAW3395_REG_PERFORMANCE         0x40   // 性能配置，R/W，Default：0x00
#define PAW3395_REG_SET_RESOLUTION      0x47   // 设置分辨率(更新CPI)，WO，Default：0x00
#define PAW3395_REG_RESOLUTION_X_LOW    0x48   // X轴低分辨率，R/W，Default：0x63
#define PAW3395_REG_RESOLUTION_X_HIGH   0x49   // X轴高分辨率，R/W，Default：0x00
#define PAW3395_REG_RESOLUTION_Y_LOW    0x4A   // Y轴低分辨率，R/W，Default：0x63
#define PAW3395_REG_RESOLUTION_Y_HIGH   0x4B   // Y轴高分辨率，R/W，Default：0x00
#define PAW3395_REG_ANGLE_SNAP          0x56   // 角度捕捉，R/W，Default：0x0D
#define PAW3395_REG_RAWDATA_OUTPUT      0x58   // 原始数据输出，RO，Default：0x00
#define PAW3395_REG_RAWDATA_STATUS      0x59   // 原始数据状态，RO，Default：0x00
#define PAW3395_REG_RIPPLE_CONTROL      0x5A   // 纹波控制，R/W，Default：0x00
#define PAW3395_REG_AXIS_CONTROL        0x5B   // 坐标系翻转，R/W，Default：0x60
#define PAW3395_REG_MOTION_CTRL         0x5C   // 运动控制，R/W，Default：0x02
#define PAW3395_REG_INV_PRODUCT_ID      0x5F   // 反向产品ID，RO，Default：0xAE
#define PAW3395_REG_RUN_DOWNSHIFT       0x77   // 运行下移，R/W，Default：0x14
#define PAW3395_REG_REST1_PERIOD        0x78   // 休息1周期，R/W，Default：0x01
#define PAW3395_REG_REST1_DOWNSHIFT     0x79   // 休息1下移，R/W，Default：0x90
#define PAW3395_REG_REST2_PERIOD        0x7A   // 休息2周期，R/W，Default：0x19
#define PAW3395_REG_REST2_DOWNSHIFT     0x7B   // 休息2下移，R/W，Default：0x5E
#define PAW3395_REG_REST3_PERIOD        0x7C   // 休息3周期，R/W，Default：0x3F
#define PAW3395_REG_RUN_DOWNSHIFT_MULT  0x7D   // 运行下移倍数，R/W，Default：0x07
#define PAW3395_REG_REST_DOWNSHIFT_MULT 0x7E   // 休息下移倍数，R/W，Default：0x55
#define PAW3395_REG_ANGLE_TUNE1         0x0577 // 角度调整1，R/W，Default：0x00
#define PAW3395_REG_ANGLE_TUNE2         0x0578 // 角度调整2使能，R/W，Default：0x00
#define PAW3395_REG_LIFT_CONFIG         0x0C4E // 抬起配置，R/W，Default：0x08
/* Add other registers as needed */

#define SPI_CS_HIGH(gpio_spec)                                                                     \
	{                                                                                          \
		gpio_pin_set_dt(gpio_spec, 0);                                                     \
	}

#define SPI_CS_LOW(gpio_spec)                                                                      \
	{                                                                                          \
		gpio_pin_set_dt(gpio_spec, 1);                                                     \
	}

#define SPI_WRITE_ADDR(spec, addr, value)                                                          \
	({                                                                                         \
		const struct spi_buf tx_buf = {                                                    \
			.buf = (uint8_t[]){(addr) | 0x80, (value)},                                \
			.len = 2,                                                                  \
		};                                                                                 \
		const struct spi_buf_set tx = {                                                    \
			.buffers = &tx_buf,                                                        \
			.count = 1,                                                                \
		};                                                                                 \
		spi_write_dt(spec, &tx);                                                           \
	})

#define SPI_READ_ADDR(spec, addr)                                                                  \
	({                                                                                         \
		int _err;              /* 局部变量存储 spi_transceive_dt 的返回值 */               \
		uint8_t _value = 0xFF; /* 读取值，失败时默认为 0xFF */                             \
		const uint8_t _tx_cmd[2] = {(addr) & 0x7F, 0xFF}; /* 发送: 读命令地址 (MSB=0) */   \
		uint8_t _rx_data[2]; /* 接收: 忽略的字节 + 实际数据字节 */                         \
                                                                                                   \
		/* 发送缓冲区设置 */                                                               \
		struct spi_buf _tx_buf = {                                                         \
			.buf = (void *)_tx_cmd, /* 指向发送数据的指针 */                           \
			.len = 2,                                                                  \
		};                                                                                 \
		struct spi_buf_set _tx = {                                                         \
			.buffers = &_tx_buf,                                                       \
			.count = 1,                                                                \
		};                                                                                 \
		/* 接收缓冲区设置 (注意 .buf 不是 const) */                                        \
		struct spi_buf _rx_buf = {                                                         \
			.buf = _rx_data, /* 指向接收缓冲区的指针 */                                \
			.len = 2,                                                                  \
		};                                                                                 \
		const struct spi_buf_set _rx = {                                                   \
			.buffers = &_rx_buf,                                                       \
			.count = 1,                                                                \
		};                                                                                 \
		/* 执行 SPI 传输 (同时发送和接收) */                                               \
		_err = spi_transceive_dt(spec, &_tx, &_rx);                                        \
		if (_err == 0) {                                                                   \
			/* 成功: 根据常见 SPI 读时序，第二个接收到的字节是所需数据 */              \
			_value = _rx_data[1];                                                      \
		} else {                                                                           \
			/* 失败: 可以选择性地记录错误 */                                           \
			/* LOG_ERR("SPI read failed for addr 0x%02X: %d", (addr) & 0x7F, _err); */ \
			/* _value 保持默认的错误值 0xFF */                                         \
		}                                                                                  \
		_value;                                                                            \
	})

/* PAW3395 Product ID value */
#define PAW3395_PRODUCT_ID 0x51 /* Verify this value with datasheet */

struct paw3395_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec cs_gpio;
	uint32_t dpi;
	bool ripple_control;
};

struct paw3395_data {
	int16_t dx;
	int16_t dy;

	int64_t sum_x_last;
	int64_t sum_y_last;

	int64_t sum_x;
	int64_t sum_y;
	bool motion;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_PAW3395_H_ */
