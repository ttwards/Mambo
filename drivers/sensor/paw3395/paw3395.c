/**
 * @file paw3395.c
 * @brief Driver for PAW3395 optical mouse sensor
 */

#include "zephyr/devicetree.h"
#define DT_DRV_COMPAT pixart_paw3395

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "paw3395.h"

LOG_MODULE_REGISTER(PAW3395, CONFIG_SENSOR_LOG_LEVEL);

/* PAW3395 register access timing requirements (in microseconds) */
#define PAW3395_T_SRAD_US 5   /* SPI Read Access Delay */
#define PAW3395_T_SWW_US  5   /* SPI Write Wait Delay */
#define PAW3395_T_CS_NS   125 /* CS assertion to first clock edge */

static int paw3395_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct paw3395_data *data = dev->data;
	const struct paw3395_config *config = dev->config;

	data->motion = SPI_READ_ADDR(&config->bus, PAW3395_REG_MOTION) != 0x00;

	if (data->motion) {
		/* Combine high and low bytes into 16-bit signed values */
		data->dx = -(int16_t)(SPI_READ_ADDR(&config->bus, PAW3395_REG_DELTA_X_L) +
				      (SPI_READ_ADDR(&config->bus, PAW3395_REG_DELTA_X_H) << 8));
		data->dy = (int16_t)(SPI_READ_ADDR(&config->bus, PAW3395_REG_DELTA_Y_L) +
				     (SPI_READ_ADDR(&config->bus, PAW3395_REG_DELTA_Y_H) << 8));

		data->sum_x += data->dx;
		data->sum_y += data->dy;

		data->sum_x_last += data->dx;
		data->sum_y_last += data->dy;
	} else {
		data->dx = 0;
		data->dy = 0;
	}
	SPI_CS_HIGH(&config->cs_gpio);
	return 0;
}

static int paw3395_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct paw3395_data *data = dev->data;
	const struct paw3395_config *config = dev->config;

	if (chan == SENSOR_CHAN_POS_DX) {
		int64_t micro = (float)data->sum_x_last * (25400000.0f / (float)config->dpi);

		val->val1 = (int32_t)(micro / 1000000LL);
		val->val2 = (int32_t)(micro % 1000000LL);

		data->sum_x_last = 0;
		return 0;
	} else if (chan == SENSOR_CHAN_POS_DY) {
		int64_t micro = (float)data->sum_y_last * (25400000.0f / (float)config->dpi);

		val->val1 = (int32_t)(micro / 1000000LL);
		val->val2 = (int32_t)(micro % 1000000LL);

		data->sum_y_last = 0;
		return 0;
	}

	return -ENOTSUP;
}

static void paw3395_enable_ripple(const struct device *dev)
{
	const struct paw3395_config *config = dev->config;
	int ret;

	uint8_t temp = SPI_READ_ADDR(&config->bus, PAW3395_REG_RIPPLE_CONTROL);
	// Set bit 7 to 1 to enable ripple
	temp |= 0x80;
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_RIPPLE_CONTROL, temp);

	return;
}

static void paw3395_dpi_set(const struct device *dev, uint16_t DPI_Num)
{
	const struct paw3395_config *config = dev->config;

	// 设置分辨率模式：X轴和Y轴分辨率均由RESOLUTION_X_LOW和RESOLUTION_X_HIGH定义
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_MOTION, 0x00);

	// 两个8位寄存器设置X轴分辨率
	int temp = (uint8_t)(((DPI_Num / 50) << 8) >> 8);
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_RESOLUTION_X_LOW, temp);
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_RESOLUTION_Y_LOW, temp);
	temp = (uint8_t)((DPI_Num / 50) >> 8);
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_RESOLUTION_X_HIGH, temp);
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_RESOLUTION_Y_HIGH, temp);

	// 更新分辨率
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_SET_RESOLUTION, 0x01);

	return;
}

static int paw3395_set_lift_off(const struct device *dev, bool height)
{
	const struct paw3395_config *config = dev->config;
	int ret;

	// 1. 将值0x0C写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x0C);
	// 2. 将值0x01写入寄存器0x4E
	SPI_WRITE_ADDR(&config->bus, 0x4E,
		       height ? 0x01 : 0x00); // 0x4E 未定义
	// 3. 将值0x00写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x00);

	return 0;
}

static int paw3395_corded_gaming(const struct device *dev)
{
	const struct paw3395_config *config = dev->config;
	// 1. 将值0x05写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x05);
	// 2. 将值0x40写入寄存器0x51
	SPI_WRITE_ADDR(&config->bus, 0x51, 0x40); // 0x51 未定义
	// 3. 将值0x40写入寄存器0x53
	SPI_WRITE_ADDR(&config->bus, 0x53, 0x40); // 0x53 未定义
	// 4. 将值0x31写入寄存器0x61
	SPI_WRITE_ADDR(&config->bus, 0x61, 0x31); // 0x61 未定义
	// 5. 将值0x0F写入寄存器0x6E
	SPI_WRITE_ADDR(&config->bus, 0x6E, 0x0F); // 0x6E 未定义
	// 6. 将值0x07写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x07);
	// 7. 将值0x32写入寄存器0x42
	SPI_WRITE_ADDR(&config->bus, 0x42, 0x2F); // 0x42 未定义
	// 8. 将值0x00写入寄存器0x43
	SPI_WRITE_ADDR(&config->bus, 0x43, 0x00); // 0x43 未定义
	// 9. 将值0x0D写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x0D);
	// 10. 将值0x12写入寄存器0x51
	SPI_WRITE_ADDR(&config->bus, 0x51, 0x12); // 0x51 未定义
	// 11. 将值0x49写入寄存器0x52
	SPI_WRITE_ADDR(&config->bus, 0x52, 0xDB); // 0x52 未定义
	// 12. 将值0x12写入寄存器0x53
	SPI_WRITE_ADDR(&config->bus, 0x53, 0x12); // 0x53 未定义
	// 13. 将值0x5B写入寄存器0x54
	SPI_WRITE_ADDR(&config->bus, 0x54, 0xDC); // 0x54 未定义
	// 14. 将值0x12写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0x12); // 0x55 未定义
	// 15. 将值0xEA写入寄存器0x56
	SPI_WRITE_ADDR(&config->bus, 0x56, 0xEA); // 0x56 未定义
	// 16. 将值0x15写入寄存器0x57
	SPI_WRITE_ADDR(&config->bus, 0x57, 0x15); // 0x57 未定义
	// 17. 将值0x2D写入寄存器0x58
	SPI_WRITE_ADDR(&config->bus, 0x58, 0x2D); // 0x58 未定义
	// 18. 将值0x00写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x00);
	// 19. 将值0x55写入寄存器0x54
	SPI_WRITE_ADDR(&config->bus, 0x54, 0x55); // 0x54 未定义
	// 20. 将值0x83写入寄存器0x40
	SPI_WRITE_ADDR(&config->bus, 0x40, 0x83);

	return 0;
}

static int paw3395_powerup(const struct device *dev)
{
	const struct paw3395_config *config = dev->config;
	int ret;

	/* Assert CS & Reset chip */
	SPI_CS_LOW(&config->cs_gpio);
	k_busy_wait(PAW3395_T_SWW_US);
	SPI_CS_HIGH(&config->cs_gpio);
	k_busy_wait(PAW3395_T_SWW_US);

	/* Write register and data */
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_POWER_UP_RESET, 0x5A);

	k_busy_wait(5000);

	// 1. 将值0x07写入寄存器0x7F (Bank Select or similar?)
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x07);
	// 2. 将值0x41写入寄存器0x40 (PAW3395_REG_PERFORMANCE)
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_PERFORMANCE, 0x41);
	// 3. 将值0x00写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x00);
	// 4. 将值0x80写入寄存器0x40 (PAW3395_REG_PERFORMANCE)
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_PERFORMANCE, 0x80);
	// 5. 将值0x0E写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x0E);
	// 6. 将值0x0D写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0x0D); // 0x55 未定义
	// 7. 将值0x1B写入寄存器0x56 (PAW3395_REG_ANGLE_SNAP)
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_ANGLE_SNAP, 0x1B);
	// 8. 将值0xE8写入寄存器0x57
	SPI_WRITE_ADDR(&config->bus, 0x57, 0xE8); // 0x57 未定义
	// 9. 将值0xD5写入寄存器0x58 (PAW3395_REG_RAWDATA_OUTPUT - 注意: 定义为 RO)
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_RAWDATA_OUTPUT, 0xD5);
	// 10. 将值0x14写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x14);
	// 11. 将值0xBC写入寄存器0x42
	SPI_WRITE_ADDR(&config->bus, 0x42, 0xBC); // 0x42 未定义
	// 12. 将值0x74写入寄存器0x43
	SPI_WRITE_ADDR(&config->bus, 0x43, 0x74); // 0x43 未定义
	// 13. 将值0x20写入寄存器0x4B (PAW3395_REG_RESOLUTION_Y_HIGH)
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_RESOLUTION_Y_HIGH, 0x20);
	// 14. 将值0x00写入寄存器0x4D
	SPI_WRITE_ADDR(&config->bus, 0x4D, 0x00); // 0x4D 未定义
	// 15. 将值0x0E写入寄存器0x53
	SPI_WRITE_ADDR(&config->bus, 0x53, 0x0E); // 0x53 未定义
	// 16. 将值0x05写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x05);
	// 17. 将值0x04写入寄存器0x44
	SPI_WRITE_ADDR(&config->bus, 0x44, 0x04); // 0x44 未定义
	// 18. 将值0x06写入寄存器0x4D
	SPI_WRITE_ADDR(&config->bus, 0x4D, 0x06); // 0x4D 未定义
	// 19. 将值0x40写入寄存器0x51
	SPI_WRITE_ADDR(&config->bus, 0x51, 0x40); // 0x51 未定义
	// 20. 将值0x40写入寄存器0x53
	SPI_WRITE_ADDR(&config->bus, 0x53, 0x40); // 0x53 未定义
	// 21. 将值0xCA写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0xCA); // 0x55 未定义
	// 22. 将值0xE8写入寄存器0x5A (PAW3395_REG_RIPPLE_CONTROL)
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_RIPPLE_CONTROL, 0xE8);
	// 23. 将值0xEA写入寄存器0x5B (PAW3395_REG_AXIS_CONTROL) - 注意: 原文是 OxEA，应为 0xEA
	SPI_WRITE_ADDR(&config->bus, PAW3395_REG_AXIS_CONTROL, 0xEA);
	// 24. 将值0x31写入寄存器0x61
	SPI_WRITE_ADDR(&config->bus, 0x61, 0x31); // 0x61 未定义
	// 25. 将值0x64写入寄存器0x62
	SPI_WRITE_ADDR(&config->bus, 0x62, 0x64); // 0x62 未定义
	// 26. 将值0xB8写入寄存器0x6D - 注意: 原文是 Ox6D，应为 0x6D
	SPI_WRITE_ADDR(&config->bus, 0x6D, 0xB8); // 0x6D 未定义
	// 27. 将值0x0F写入寄存器0x6E
	SPI_WRITE_ADDR(&config->bus, 0x6E, 0x0F); // 0x6E 未定义
	// 28. 将值0x02写入寄存器0x70
	SPI_WRITE_ADDR(&config->bus, 0x70, 0x02); // 0x70 未定义
	// 29. 将值0x2A写入寄存器0x4A
	SPI_WRITE_ADDR(&config->bus, 0x4A, 0x2A); // 0x4A 未定义
	// 30. 将值0x26写入寄存器0x60
	SPI_WRITE_ADDR(&config->bus, 0x60, 0x26); // 0x60 未定义
	// 31. 将值0x06写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x06);
	// 32. 将值0x70写入寄存器0x6D
	SPI_WRITE_ADDR(&config->bus, 0x6D, 0x70); // 0x6D 未定义
	// 33. 将值0x60写入寄存器0x6E
	SPI_WRITE_ADDR(&config->bus, 0x6E, 0x60); // 0x6E 未定义
	// 34. 将值0x04写入寄存器0x6F
	SPI_WRITE_ADDR(&config->bus, 0x6F, 0x04); // 0x6F 未定义
	// 35. 将值0x02写入寄存器0x53
	SPI_WRITE_ADDR(&config->bus, 0x53, 0x02); // 0x53 未定义
	// 36. 将值0x11写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0x11); // 0x55 未定义
	// 37. 将值0x01写入寄存器0x7A
	SPI_WRITE_ADDR(&config->bus, 0x7A, 0x01); // 0x7A 未定义
	// 38. 将值0x51写入寄存器0x7D
	SPI_WRITE_ADDR(&config->bus, 0x7D, 0x51); // 0x7D 未定义
	// 39. 将值0x07写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x07);
	// 40. 将值0x10写入寄存器0x41
	SPI_WRITE_ADDR(&config->bus, 0x41, 0x10); // 0x41 未定义
	// 41. 将值0x32写入寄存器0x42
	SPI_WRITE_ADDR(&config->bus, 0x42, 0x32); // 0x42 未定义
	// 42. 将值0x00写入寄存器0x43
	SPI_WRITE_ADDR(&config->bus, 0x43, 0x00); // 0x43 未定义
	// 43. 将值0x08写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x08);
	// 44. 将值0x4F写入寄存器0x71
	SPI_WRITE_ADDR(&config->bus, 0x71, 0x4F); // 0x71 未定义
	// 45. 将值0x09写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x09);
	// 46. 将值0x1F写入寄存器0x62
	SPI_WRITE_ADDR(&config->bus, 0x62, 0x1F); // 0x62 未定义
	// 47. 将值0x1F写入寄存器0x63
	SPI_WRITE_ADDR(&config->bus, 0x63, 0x1F); // 0x63 未定义
	// 48. 将值0x03写入寄存器0x65
	SPI_WRITE_ADDR(&config->bus, 0x65, 0x03); // 0x65 未定义
	// 49. 将值0x03写入寄存器0x66
	SPI_WRITE_ADDR(&config->bus, 0x66, 0x03); // 0x66 未定义
	// 50. 将值0x1F写入寄存器0x67
	SPI_WRITE_ADDR(&config->bus, 0x67, 0x1F); // 0x67 未定义
	// 51. 将值0x1F写入寄存器0x68
	SPI_WRITE_ADDR(&config->bus, 0x68, 0x1F); // 0x68 未定义
	// 52. 将值0x03写入寄存器0x69
	SPI_WRITE_ADDR(&config->bus, 0x69, 0x03); // 0x69 未定义
	// 53. 将值0x03写入寄存器0x6A
	SPI_WRITE_ADDR(&config->bus, 0x6A, 0x03); // 0x6A 未定义
	// 54. 将值0x1F写入寄存器0x6C
	SPI_WRITE_ADDR(&config->bus, 0x6C, 0x1F); // 0x6C 未定义
	// 55. 将值0x1F写入寄存器0x6D
	SPI_WRITE_ADDR(&config->bus, 0x6D, 0x1F); // 0x6D 未定义
	// 56. 将值0x04写入寄存器0x51
	SPI_WRITE_ADDR(&config->bus, 0x51, 0x04); // 0x51 未定义
	// 57. 将值0x20写入寄存器0x53
	SPI_WRITE_ADDR(&config->bus, 0x53, 0x20); // 0x53 未定义
	// 58. 将值0x20写入寄存器0x54
	SPI_WRITE_ADDR(&config->bus, 0x54, 0x20); // 0x54 未定义
	// 59. 将值0x0C写入寄存器0x71
	SPI_WRITE_ADDR(&config->bus, 0x71, 0x0C); // 0x71 未定义
	// 60. 将值0x07写入寄存器0x72
	SPI_WRITE_ADDR(&config->bus, 0x72, 0x07); // 0x72 未定义
	// 61. 将值0x07写入寄存器0x73
	SPI_WRITE_ADDR(&config->bus, 0x73, 0x07); // 0x73 未定义
	// 62. 将值0x0A写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x0A);
	// 63. 将值0x14写入寄存器0x4A
	SPI_WRITE_ADDR(&config->bus, 0x4A, 0x14); // 0x4A 未定义
	// 64. 将值0x14写入寄存器0x4C
	SPI_WRITE_ADDR(&config->bus, 0x4C, 0x14); // 0x4C 未定义
	// 65. 将值0x19写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0x19); // 0x55 未定义
	// 66. 将值0x14写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x14);
	// 67. 将值0x30写入寄存器0x4B
	SPI_WRITE_ADDR(&config->bus, 0x4B, 0x30); // 0x4B 未定义
	// 68. 将值0x03写入寄存器0x4C
	SPI_WRITE_ADDR(&config->bus, 0x4C, 0x03); // 0x4C 未定义
	// 69. 将值0x0B写入寄存器0x61
	SPI_WRITE_ADDR(&config->bus, 0x61, 0x0B); // 0x61 未定义
	// 70. 将值0x0A写入寄存器0x62
	SPI_WRITE_ADDR(&config->bus, 0x62, 0x0A); // 0x62 未定义
	// 71. 将值0x02写入寄存器0x63
	SPI_WRITE_ADDR(&config->bus, 0x63, 0x02); // 0x63 未定义
	// 72. 将值0x15写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x15);
	// 73. 将值0x02写入寄存器0x4C
	SPI_WRITE_ADDR(&config->bus, 0x4C, 0x02); // 0x4C 未定义
	// 74. 将值0x02写入寄存器0x56
	SPI_WRITE_ADDR(&config->bus, 0x56, 0x02); // 0x56 未定义
	// 75. 将值0x91写入寄存器0x41
	SPI_WRITE_ADDR(&config->bus, 0x41, 0x91); // 0x41 未定义
	// 76. 将值0x0A写入寄存器0x4D
	SPI_WRITE_ADDR(&config->bus, 0x4D, 0x0A); // 0x4D 未定义
	// 77. 将值0x0C写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x0C);
	// 78. 将值0x10写入寄存器0x4A
	SPI_WRITE_ADDR(&config->bus, 0x4A, 0x10); // 0x4A 未定义
	// 79. 将值0x0C写入寄存器0x4B
	SPI_WRITE_ADDR(&config->bus, 0x4B, 0x0C); // 0x4B 未定义
	// 80. 将值0x40写入寄存器0x4C
	SPI_WRITE_ADDR(&config->bus, 0x4C, 0x40); // 0x4C 未定义
	// 81. 将值0x25写入寄存器0x41
	SPI_WRITE_ADDR(&config->bus, 0x41, 0x25); // 0x41 未定义
	// 82. 将值0x18写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0x18); // 0x55 未定义
	// 83. 将值0x14写入寄存器0x56
	SPI_WRITE_ADDR(&config->bus, 0x56, 0x14); // 0x56 未定义
	// 84. 将值0x0A写入寄存器0x49
	SPI_WRITE_ADDR(&config->bus, 0x49, 0x0A); // 0x49 未定义
	// 85. 将值0x00写入寄存器0x42
	SPI_WRITE_ADDR(&config->bus, 0x42, 0x00); // 0x42 未定义
	// 86. 将值0x2D写入寄存器0x43
	SPI_WRITE_ADDR(&config->bus, 0x43, 0x2D); // 0x43 未定义
	// 87. 将值0x0C写入寄存器0x44
	SPI_WRITE_ADDR(&config->bus, 0x44, 0x0C); // 0x44 未定义
	// 88. 将值0x1A写入寄存器0x54
	SPI_WRITE_ADDR(&config->bus, 0x54, 0x1A); // 0x54 未定义
	// 89. 将值0x0D写入寄存器0x5A
	SPI_WRITE_ADDR(&config->bus, 0x5A, 0x0D); // 0x5A 未定义
	// 90. 将值0x1E写入寄存器0x5F
	SPI_WRITE_ADDR(&config->bus, 0x5F, 0x1E); // 0x5F 未定义
	// 91. 将值0x05写入寄存器0x5B
	SPI_WRITE_ADDR(&config->bus, 0x5B, 0x05); // 0x5B 未定义
	// 92. 将值0x0F写入寄存器0x5E
	SPI_WRITE_ADDR(&config->bus, 0x5E, 0x0F); // 0x5E 未定义
	// 93. 将值0x0D写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x0D);
	// 94. 将值0xDD写入寄存器0x48
	SPI_WRITE_ADDR(&config->bus, 0x48, 0xDD); // 0x48 未定义
	// 95. 将值0x03写入寄存器0x4F
	SPI_WRITE_ADDR(&config->bus, 0x4F, 0x03); // 0x4F 未定义
	// 96. 将值0x49写入寄存器0x52
	SPI_WRITE_ADDR(&config->bus, 0x52, 0x49); // 0x52 未定义
	// 97. 将值0x00写入寄存器0x51
	SPI_WRITE_ADDR(&config->bus, 0x51, 0x00); // 0x51 未定义
	// 98. 将值0x5B写入寄存器0x54
	SPI_WRITE_ADDR(&config->bus, 0x54, 0x5B); // 0x54 未定义
	// 99. 将值0x00写入寄存器0x53
	SPI_WRITE_ADDR(&config->bus, 0x53, 0x00); // 0x53 未定义
	// 100. 将值0x64写入寄存器0x56
	SPI_WRITE_ADDR(&config->bus, 0x56, 0x64); // 0x56 未定义
	// 101. 将值0x00写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0x00); // 0x55 未定义
	// 102. 将值0xA5写入寄存器0x58
	SPI_WRITE_ADDR(&config->bus, 0x58, 0xA5); // 0x58 未定义
	// 103. 将值0x02写入寄存器0x57
	SPI_WRITE_ADDR(&config->bus, 0x57, 0x02); // 0x57 未定义
	// 104. 将值0x29写入寄存器0x5A
	SPI_WRITE_ADDR(&config->bus, 0x5A, 0x29); // 0x5A 未定义
	// 105. 将值0x47写入寄存器0x5B
	SPI_WRITE_ADDR(&config->bus, 0x5B, 0x47); // 0x5B 未定义
	// 106. 将值0x81写入寄存器0x5C
	SPI_WRITE_ADDR(&config->bus, 0x5C, 0x81); // 0x5C 未定义
	// 107. 将值0x40写入寄存器0x5D
	SPI_WRITE_ADDR(&config->bus, 0x5D, 0x40); // 0x5D 未定义
	// 108. 将值0xDC写入寄存器0x71
	SPI_WRITE_ADDR(&config->bus, 0x71, 0xDC); // 0x71 未定义
	// 109. 将值0x07写入寄存器0x70
	SPI_WRITE_ADDR(&config->bus, 0x70, 0x07); // 0x70 未定义
	// 110. 将值0x00写入寄存器0x73
	SPI_WRITE_ADDR(&config->bus, 0x73, 0x00); // 0x73 未定义
	// 111. 将值0x08写入寄存器0x72
	SPI_WRITE_ADDR(&config->bus, 0x72, 0x08); // 0x72 未定义
	// 112. 将值0xDC写入寄存器0x75
	SPI_WRITE_ADDR(&config->bus, 0x75, 0xDC); // 0x75 未定义
	// 113. 将值0x07写入寄存器0x74
	SPI_WRITE_ADDR(&config->bus, 0x74, 0x07); // 0x74 未定义
	// 114. 将值0x00写入寄存器0x77
	SPI_WRITE_ADDR(&config->bus, 0x77, 0x00); // 0x77 未定义
	// 115. 将值0x08写入寄存器0x76
	SPI_WRITE_ADDR(&config->bus, 0x76, 0x08); // 0x76 未定义
	// 116. 将值0x10写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x10);
	// 117. 将值0xD0写入寄存器0x4C
	SPI_WRITE_ADDR(&config->bus, 0x4C, 0xD0); // 0x4C 未定义
	// 118. 将值0x00写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x00);
	// 119. 将值0x63写入寄存器0x4F
	SPI_WRITE_ADDR(&config->bus, 0x4F, 0x63); // 0x4F 未定义
	// 120. 将值0x00写入寄存器0x4E
	SPI_WRITE_ADDR(&config->bus, 0x4E, 0x00); // 0x4E 未定义
	// 121. 将值0x63写入寄存器0x52
	SPI_WRITE_ADDR(&config->bus, 0x52, 0x63); // 0x52 未定义
	// 122. 将值0x00写入寄存器0x51
	SPI_WRITE_ADDR(&config->bus, 0x51, 0x00); // 0x51 未定义
	// 123. 将值0x54写入寄存器0x54
	SPI_WRITE_ADDR(&config->bus, 0x54, 0x54); // 0x54 未定义
	// 124. 将值0x10写入寄存器0x5A
	SPI_WRITE_ADDR(&config->bus, 0x5A, 0x10); // 0x5A 未定义
	// 125. 将值0x4F写入寄存器0x77
	SPI_WRITE_ADDR(&config->bus, 0x77, 0x4F); // 0x77 未定义
	// 126. 将值0x01写入寄存器0x47
	SPI_WRITE_ADDR(&config->bus, 0x47, 0x01); // 0x47 未定义
	// 127. 将值0x40写入寄存器0x5B
	SPI_WRITE_ADDR(&config->bus, 0x5B, 0x40); // 0x5B 未定义
	// 128. 将值0x60写入寄存器0x64
	SPI_WRITE_ADDR(&config->bus, 0x64, 0x60); // 0x64 未定义
	// 129. 将值0x06写入寄存器0x65
	SPI_WRITE_ADDR(&config->bus, 0x65, 0x06); // 0x65 未定义
	// 130. 将值0x13写入寄存器0x66
	SPI_WRITE_ADDR(&config->bus, 0x66, 0x13); // 0x66 未定义
	// 131. 将值0x0F写入寄存器0x67
	SPI_WRITE_ADDR(&config->bus, 0x67, 0x0F); // 0x67 未定义
	// 132. 将值0x01写入寄存器0x78
	SPI_WRITE_ADDR(&config->bus, 0x78, 0x01); // 0x78 未定义
	// 133. 将值0x9C写入寄存器0x79
	SPI_WRITE_ADDR(&config->bus, 0x79, 0x9C); // 0x79 未定义
	// 134. 将值0x00写入寄存器0x40
	SPI_WRITE_ADDR(&config->bus, 0x40, 0x00); // 0x40 未定义
	// 135. 将值0x02写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0x02); // 0x55 未定义
	// 136. 将值0x70写入寄存器0x23
	SPI_WRITE_ADDR(&config->bus, 0x23, 0x70); // 0x23 未定义
	// 137. 将值0x01写入寄存器0x22
	SPI_WRITE_ADDR(&config->bus, 0x22, 0x01); // 0x22 未定义

	k_busy_wait(1000);

	for (int retry = 0; retry < 60; retry++) {
		if (SPI_READ_ADDR(&config->bus, 0x6C) == 0x80) {
			goto success;
		}
		k_busy_wait(1000);
	}

	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x14);
	SPI_WRITE_ADDR(&config->bus, 0x6C, 0x00);
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x00);

success:
	// 138. 将值0x70写入寄存器0x22
	SPI_WRITE_ADDR(&config->bus, 0x22, 0x70); // 0x22 未定义
	// 139. 将值0x01写入寄存器0x22
	SPI_WRITE_ADDR(&config->bus, 0x22, 0x01); // 0x22 未定义
	// 140. 将值0x00写入寄存器0x22
	SPI_WRITE_ADDR(&config->bus, 0x22, 0x00); // 0x22 未定义
	// 141. 将值0x00写入寄存器0x55
	SPI_WRITE_ADDR(&config->bus, 0x55, 0x00); // 0x55 未定义
	// 142. 将值0x07写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x07);
	// 143. 将值0x40写入寄存器0x40
	SPI_WRITE_ADDR(&config->bus, 0x40, 0x40); // 0x40 未定义
	// 144. 将值0x00写入寄存器0x7F
	SPI_WRITE_ADDR(&config->bus, 0x7F, 0x00);

	for (int reg = 0x02; reg <= 0x06; reg++) {
		SPI_READ_ADDR(&config->bus, reg);
	}

	SPI_CS_HIGH(&config->cs_gpio)

	if (ret < 0) {
		// LOG_INF("Failed to write register %d", ret);
		return ret;
	}

	return 0;
}

static int paw3395_init(const struct device *dev)
{
	const struct paw3395_config *config = dev->config;
	uint8_t chip_id;
	int ret;

	/* Initialize CS GPIO */
	if (!gpio_is_ready_dt(&config->cs_gpio)) {
		LOG_ERR("CS GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure CS GPIO pin");
		return ret;
	}

	/* Check if SPI bus is ready */
	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus not ready");
		return -ENODEV;
	}

	/* Read chip ID to verify device */
	k_sleep(K_MSEC(50)); /* Initial power-up delay */

	paw3395_powerup(dev);
	paw3395_corded_gaming(dev);
	paw3395_dpi_set(dev, config->dpi);

	paw3395_set_lift_off(dev, 1);

	if (config->ripple_control) {
		paw3395_enable_ripple(dev);
	}

	chip_id = SPI_READ_ADDR(&config->bus, PAW3395_REG_PRODUCT_ID);
	if (chip_id != PAW3395_PRODUCT_ID) {
		LOG_ERR("Wrong chip ID: %02x, expected %02x", chip_id, PAW3395_PRODUCT_ID);
		// return -ENODEV;
	}
	LOG_DBG("Chip ID: %02x", chip_id);

	LOG_DBG("PAW3395 initialized, ID: %02x", chip_id);

	/* Initialize sensor with default settings */
	/* Add initialization sequence according to datasheet */

	return 0;
}

static void paw3395_thread(void *arg1, void *arg2, void *arg3)
{
	const struct device *dev = arg1;
	struct paw3395_data *data = dev->data;
	const struct paw3395_config *config = dev->config;

	while (1) {
		k_sleep(K_MSEC(10));

		if (paw3395_sample_fetch(dev, SENSOR_CHAN_ALL) < 0) {
			LOG_ERR("Failed to read raw data");
			continue;
		}
	}
}

static const struct sensor_driver_api paw3395_driver_api = {
	.sample_fetch = paw3395_sample_fetch,
	.channel_get = paw3395_channel_get,
};

#define PAW3395_INIT(inst)                                                                         \
	static struct paw3395_data paw3395_data_##inst = {                                         \
		.dx = 0,                                                                           \
		.dy = 0,                                                                           \
		.motion = false,                                                                   \
		.sum_x = 0,                                                                        \
		.sum_x_last = 0,                                                                   \
		.sum_y_last = 0,                                                                   \
		.sum_y = 0,                                                                        \
	};                                                                                         \
                                                                                                   \
	static const struct paw3395_config paw3395_config_##inst = {                               \
		.bus = SPI_DT_SPEC_INST_GET(inst,                                                  \
					    SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_MODE_CPOL | \
						    SPI_MODE_CPHA | SPI_TRANSFER_MSB,              \
					    PAW3395_T_SWW_US),                                     \
		.cs_gpio = GPIO_DT_SPEC_INST_GET(inst, cs_gpios),                                  \
		.dpi = DT_PROP(DT_DRV_INST(inst), dpi),                                            \
		.ripple_control = DT_PROP(DT_DRV_INST(inst), ripple),                              \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, paw3395_init, NULL, &paw3395_data_##inst,                      \
			      &paw3395_config_##inst, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
			      &paw3395_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PAW3395_INIT)
