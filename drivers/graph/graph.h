#include <zephyr/devicetree.h>
#include <zephyr/device.h>

typedef enum {
	STRING,
	INTEGER,
	FLOAT,
	DOUBLE,
	RAW
} VarType;

typedef struct {
	VarType type;
	int16_t size;
	void *ptr;
} feedbackVar;

#define DT_GRAPH_DEFINE(node_id)                                                                   \
	DEVICE_DT_DEFINE(node_id, init_fn, pm, data, config, level, prio, api, __VA_ARGS__)
