/*
 * Copyright (c) 2024 Librgod
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT wit_hwt906

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/uart.h>

#define DATA(idx) (short)((short)buf[idx + 1] << 8 | buf[idx]) // 组合高低字节

#define WIT_BUF_SIZE 128

struct wit_hwt906_data {
	const struct device *dev; // 指向自身的指针

	/*环形缓冲区*/
	struct ring_buf rx_ringbuf;
	uint8_t rx_buf[WIT_BUF_SIZE]; // 实际存数据的数组

	/*缓存解析后的数据(后面统一单位)*/
	struct {
		uint8_t time[8];  // 时间戳，对应 0x50 包
		int16_t acc[3];   // 加速度，对应 0x51 包: Ax, Ay, Az
		int16_t gyro[3];  // 角速度，对应 0x52 包: Wx, Wy, Wz
		int16_t angle[3]; // 角度，对应 0x53 包: Roll, Pitch, Yaw
		int16_t mag[3];   // 磁场，对应 0x54 包: Hx, Hy, Hz
		int16_t temp;     // 温度，对应 0x51 包: TL, TH
		int16_t Vol;      // 电压，对应 0x52 包: VL, VH
		int16_t Version;  // 版本号，对应 0x53 包: VerL, VerH
	} raw;

	uint32_t total_bytes_dropped;
	uint32_t overflow_count;
};

static void wit_hwt906_uart_cb(const struct device *uart_dev, void *user_data)
{
	const struct device *sensor_dev = user_data;
	struct wit_hwt906_data *data = sensor_dev->data;

	if (!uart_irq_update(uart_dev)) {
		return;
	}

	if (uart_irq_rx_ready(uart_dev)) {
		uint8_t buffer[64];
		// 从硬件FIFO中读取数据到临时缓冲区
		int len = uart_fifo_read(uart_dev, buffer, sizeof(buffer));

		if (len > 0) {
			uint32_t space = ring_buf_space_get(&data->rx_ringbuf);

			/* 如果空间不足，丢弃最老的数据 */
			if (space < len) {
				uint32_t to_discard = len - space;
				uint8_t dummy[64];

				/* 分块丢弃 */
				while (to_discard > 0) {
					uint32_t chunk = MIN(to_discard, sizeof(dummy));
					uint32_t removed =
						ring_buf_get(&data->rx_ringbuf, dummy, chunk);
					to_discard -= removed;
					data->total_bytes_dropped += removed;

					if (removed == 0)
						break;
				}

				data->overflow_count++;
			}

			/* 写入新数据 */
			ring_buf_put(&data->rx_ringbuf, buffer, len);
		}
	}
}

/* 从设备树配置里获取 UART 设备的指针 */
struct wit_hwt906_config {
	const struct device *uart_dev;
};

static int wit_hwt906_init(const struct device *dev)
{
	const struct wit_hwt906_config *cfg = dev->config;
	struct wit_hwt906_data *data = dev->data;

	printk("[Driver] WIT HWT906 initializing...\n");

	/* 初始化 Ring Buffer */
	ring_buf_init(&data->rx_ringbuf, sizeof(data->rx_buf), data->rx_buf);

	if (!device_is_ready(cfg->uart_dev)) {
		printk("[Driver] Error: UART device not ready\n");
		return -ENODEV;
	}

	uart_irq_callback_user_data_set(cfg->uart_dev, wit_hwt906_uart_cb, (void *)dev);

	/* 开启接收中断 */
	uart_irq_rx_enable(cfg->uart_dev);

	printk("[Driver] UART interrupts enabled.\n");
	return 0;
}

static int wit_hwt906_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct wit_hwt906_data *data = dev->data;
	uint8_t buf[11]; // 临时存一个包
	int processed = 0;

	/* 循环处理，直到仓库里的数据不足 11 个字节 */
	while (ring_buf_size_get(&data->rx_ringbuf) >= 11) {

		/* 先偷看11个字节，不从仓库移除 */
		ring_buf_peek(&data->rx_ringbuf, buf, 11);

		/* 检查包头 0x55 */
		if (buf[0] != 0x55) {
			/* 包头不对，说明错位了。丢弃 1 个字节，下次循环再试试下一位 */
			uint8_t dummy;
			ring_buf_get(&data->rx_ringbuf, &dummy, 1);
			continue;
		}

		/* 计算校验和 (前10字节之和 == 第11字节) */
		uint8_t sum = 0;
		for (int i = 0; i < 10; i++) {
			sum = (uint8_t)(sum + buf[i]); // 强制转换为 uint8_t，只保留低8位
		}

		if (sum != buf[10]) {
			/* 校验失败，说明可能是偶然出现的 0x55，或者是坏数据。丢弃 1 字节重试 */
			uint8_t dummy;
			ring_buf_get(&data->rx_ringbuf, &dummy, 1);
			continue;
		}

		/* 校验通过！这真的是一个完整的数据包。现在正式从仓库取出。 */
		ring_buf_get(&data->rx_ringbuf, buf, 11);

		// printk("Pkg: 0x%02X\n", buf[1]);

		/* 解析数据 (低8位在前，高8位在后) */
		switch (buf[1]) {
		case 0x50: // 时间包
			for (int i = 0; i < 8; i++) {
				data->raw.time[i] = buf[i + 2];
			}
			processed++;
			break;
		case 0x51:                          // 加速度包
			data->raw.acc[0] = DATA(2); // AxL, AxH
			data->raw.acc[1] = DATA(4); // AyL, AyH
			data->raw.acc[2] = DATA(6); // AzL, AzH
			data->raw.temp = DATA(8);   // TL, TH
			processed++;
			break;
		case 0x52:                           // 角速度包
			data->raw.gyro[0] = DATA(2); // WxL, WxH
			data->raw.gyro[1] = DATA(4); // WyL, WyH
			data->raw.gyro[2] = DATA(6); // WzL, WzH
			data->raw.Vol = DATA(8);     // VL, VH
			processed++;
			break;
		case 0x53:                            // 角度包
			data->raw.angle[0] = DATA(2); // RollL, RollH
			data->raw.angle[1] = DATA(4); // PitchL, PitchH
			data->raw.angle[2] = DATA(6); // YawL, YawH
			data->raw.Version = DATA(8);  // VerL, VerH
			processed++;
			break;
		case 0x54:                          // 磁场包
			data->raw.mag[0] = DATA(2); // HxL, HxH
			data->raw.mag[1] = DATA(4); // HyL, HyH
			data->raw.mag[2] = DATA(6); // HzL, HzH
			data->raw.temp = DATA(8);   // TL, TH
			processed++;
			break;
		default:
			// 其他类型的包我们暂时不关心
			break;
		}
	}

	/* 如果处理了至少一个包，返回成功；否则返回 EAGAIN (稍后再试) */
	if (processed == 0) {
		return -EAGAIN;
	}
	return 0;
}

static void wit_convert_acc(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 32768 * 16 * 9.80665
	sensor_value_from_double(val, raw / 32768.0 * 16.0 * 9.80665);
}

static void wit_convert_gyro(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 32768 * 2000 * (PI / 180) -> 转为弧度/秒
	// 2000度/秒 约等于 34.906585 弧度/秒
	sensor_value_from_double(val, raw / 32768.0 * 34.906585);
}

static void wit_convert_angle(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 32768 * 180 (度)
	sensor_value_from_double(val, raw / 32768.0 * 180.0);
}

static void wit_convert_temp(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 100 (摄氏度)
	sensor_value_from_double(val, raw / 100.0);
}

static void wit_convert_voltage(struct sensor_value *val, int16_t raw)
{
	// 公式: raw / 100 (伏特)
	sensor_value_from_double(val, raw / 100.0);
}

static int wit_hwt906_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct wit_hwt906_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ: // 支持加速度接口
		wit_convert_acc(&val[0], data->raw.acc[0]);
		wit_convert_acc(&val[1], data->raw.acc[1]);
		wit_convert_acc(&val[2], data->raw.acc[2]);
		break;

	case SENSOR_CHAN_GYRO_XYZ: // 支持角速度接口
		wit_convert_gyro(&val[0], data->raw.gyro[0]);
		wit_convert_gyro(&val[1], data->raw.gyro[1]);
		wit_convert_gyro(&val[2], data->raw.gyro[2]);
		break;

	case SENSOR_CHAN_ROTATION: // 支持欧拉角接口 (Roll, Pitch, Yaw)
		wit_convert_angle(&val[0], data->raw.angle[0]);
		wit_convert_angle(&val[1], data->raw.angle[1]);
		wit_convert_angle(&val[2], data->raw.angle[2]);
		break;

	case SENSOR_CHAN_MAGN_XYZ: // 支持磁场接口
		sensor_value_from_double(&val[0], data->raw.mag[0]);
		sensor_value_from_double(&val[1], data->raw.mag[1]);
		sensor_value_from_double(&val[2], data->raw.mag[2]);
		break;

	case SENSOR_CHAN_DIE_TEMP: // 支持芯片温度接口
		wit_convert_temp(&val[0], data->raw.temp);
		break;

	case SENSOR_CHAN_VOLTAGE: // Zephyr 标准电压通道
		wit_convert_voltage(&val[0], data->raw.Vol);
		break;

	/* 新增：时间戳通道 (使用 PRIV_START 自定义) */
	case SENSOR_CHAN_PRIV_START: // 自定义通道起始位置
		// 时间戳格式: YY MM DD hh mm ss ms ms，这里只使用时、分、秒、毫秒
		val[0].val1 = data->raw.time[3];                              // hh
		val[1].val1 = data->raw.time[4];                              // mm
		val[2].val1 = data->raw.time[5];                              // ss
		val[3].val1 = (data->raw.time[7] << 8 | (data->raw.time[6])); // ms
		break;

	/* 新增：版本号通道 */
	case SENSOR_CHAN_PRIV_START + 1:
		val[0].val1 = data->raw.Version;
		val[0].val2 = 0;
		break;

	default:
		return -ENOTSUP; // 不支持的通道返回错误
	}

	return 0;
}

static const struct sensor_driver_api wit_hwt906_api = {
	.sample_fetch = wit_hwt906_sample_fetch,
	.channel_get = wit_hwt906_channel_get,
};

/* 实例化 Config 结构体 (从设备树里拿 UART 设备) */
#define WIT_HWT906_CONFIG_DEFINE(inst)                                                             \
	static const struct wit_hwt906_config wit_hwt906_config_##inst = {                         \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
	};

/* 实例化 Data 结构体 */
#define WIT_HWT906_DATA_DEFINE(inst) static struct wit_hwt906_data wit_hwt906_data_##inst;

/* 初始化宏 */
#define WIT_HWT906_DEFINE(inst)                                                                    \
	WIT_HWT906_CONFIG_DEFINE(inst)                                                             \
	WIT_HWT906_DATA_DEFINE(inst)                                                               \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, wit_hwt906_init, NULL, &wit_hwt906_data_##inst,         \
				     &wit_hwt906_config_##inst, POST_KERNEL,                       \
				     CONFIG_SENSOR_INIT_PRIORITY, &wit_hwt906_api);

DT_INST_FOREACH_STATUS_OKAY(WIT_HWT906_DEFINE)
