#include "graph.h"
#include <sys/_stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#define DT_DRV_COMPAT ares_rttgraph

struct rtt_graph_data {
	feedbackVar vars[16];
	int16_t num;
};

struct rtt_graph_cfg {
	const char name[12];
	const struct device *dev;
	int time_gap_ms;
};

static void rtt_graph_init(const struct device *dev);
static int rtt_graph_add(const struct device *dev, feedbackVar var);
static void rtt_graph_stop(const struct device *dev);

#define DT_RTT_GRAPH_DEFINE_INST(inst, ...) DT_GRAPH_DEFINE(DT_DRV_INST(inst), __VA_ARGS__)

#define RTT_GRAPH_DEFINE_INST(node_id)                                                             \
	DT_RTT_GRAPH_DEFINE_INST(inst, rtt_graph_init, NULL, &rtt_graph_data_##inst,               \
				 &rtt_graph_cfg_##inst, POST_KERNEL, CONFIG_RTT_GRAPH_INIT_PRIO,   \
				 &rtt_graph_api_funcs);

#define RTT_GRAPH_DATA_INST(inst)                                                                  \
	struct rtt_graph_data rtt_graph_data_##inst = {                                            \
		.num = 0,                                                                          \
	};

#define RTT_GRAPH_CONFIG_INST(inst)                                                                \
	struct rtt_graph_cfg rtt_graph_cfg_##inst = {                                              \
		.name = DT_PROP(DT_DRV_INST(inst), name),                                          \
		.dev = DEVICE_DT_GET(DT_PROP(DT_DRV_INST(inst), phy)),                             \
		.time_gap_ms = DT_PROP(DT_DRV_INST(inst), time_gap_ms),                            \
	};

#define RTT_GRAPH_INST(inst)                                                                       \
	RTT_GRAPH_CONFIG_INST(inst)                                                                \
	RTT_GRAPH_DATA_INST(inst)                                                                  \
	RTT_GRAPH_DEFINE_INST(inst)
