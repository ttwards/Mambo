
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/motor.h>
#include <zephyr/drivers/gpio.h>

/* Devicetree */
// #define UART_NODE DT_ALIAS(vofa)
// const struct device *uart_dev = DEVICE_DT_GET(UART_NODE);

#define ACCEL_NODE DT_NODELABEL(bmi08x_accel)
const struct device *accel_dev = DEVICE_DT_GET(ACCEL_NODE);

#define GYRO_NODE DT_NODELABEL(bmi08x_gyro)
const struct device *gyro_dev = DEVICE_DT_GET(GYRO_NODE);

#define BUTTON_NODE DT_CHOSEN(zephyr_button)
const struct device *button_dev = DEVICE_DT_GET(BUTTON_NODE);

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(BUTTON_NODE, gpios, {0});
static struct gpio_callback button_cb_data;
