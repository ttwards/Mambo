#ifndef JUSTFLOAT_H
#define JUSTFLOAT_H

#include <stdint.h>
#include <zephyr/drivers/uart.h>

#define aresMaxChannel 24

enum JF_Types {
	PTR_INT = 0,
	PTR_FLOAT = 1,
	PTR_DOUBLE = 2,
	PTR_INT8 = 3,
	PTR_INT16 = 4,
	PTR_UINT8 = 5,
	PTR_UINT16 = 6,
	PTR_UINT = 7,
	RAW = 8,
};

struct JFData {
	struct device *uart_dev;

	void *data_ptr[aresMaxChannel];

	float fdata[aresMaxChannel];

	enum JF_Types types[aresMaxChannel];

	int channel;
};

struct JFData *jf_send_init(const struct device *uart_dev, int delay);
void jf_channel_add(struct JFData *data, void *value, enum JF_Types type);

#endif
