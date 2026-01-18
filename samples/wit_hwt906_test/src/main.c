#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_ALIAS(my_imu));

	if (!device_is_ready(dev)) {
		printk("Device not ready\n");
		return 0;
	}

	printk("Start polling HWT906...\n");

	struct sensor_value time[4];
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value angle[3];
	struct sensor_value mag[3];

	while (1) {
		if (sensor_sample_fetch(dev) < 0) {
			k_sleep(K_MSEC(10));
			continue;
		}

		sensor_channel_get(dev, SENSOR_CHAN_PRIV_START, time);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		sensor_channel_get(dev, SENSOR_CHAN_ROTATION, angle);
		sensor_channel_get(dev, SENSOR_CHAN_MAGN_XYZ, mag);

		printk("Time: %02d:%02d:%02d.%03d | ", time[0].val1, time[1].val1, time[2].val1,
		       time[3].val1);
		printk("Acc: X=%.2f Y=%.2f Z=%.2f | ", sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]), sensor_value_to_double(&accel[2]));
		printk("Gyro: X=%.2f Y=%.2f Z=%.2f | ", sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
		printk("Angle: R=%.2f P=%.2f Y=%.2f | ", sensor_value_to_double(&angle[0]),
		       sensor_value_to_double(&angle[1]), sensor_value_to_double(&angle[2]));
		printk("Mag: X=%.2f Y=%.2f Z=%.2f\n", sensor_value_to_double(&mag[0]),
		       sensor_value_to_double(&mag[1]), sensor_value_to_double(&mag[2]));

		k_sleep(K_MSEC(300));
	}
}
