
#include <zephyr/device.h>

/* Devicetree */
#define ACCEL_NODE DT_NODELABEL(bmi08x_accel)
const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);

#define GYRO_NODE DT_NODELABEL(bmi08x_gyro)
const struct device *gyro_dev = DEVICE_DT_GET(GYRO_NODE);

#define MAG_NODE DT_NODELABEL(ist8310)
const struct device *mag_dev = DEVICE_DT_GET(MAG_NODE);
