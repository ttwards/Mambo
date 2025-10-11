#ifndef ARES_COMM_H__
#define ARES_COMM_H__

#include <ares/protocol/ares_protocol.h>
#include <ares/interface/ares_interface.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>

static inline int ares_bind_interface(struct AresInterface *interface,
				      struct AresProtocol *protocol)
{
	// Notice that we initialize interface before protocol
	if (interface == NULL || protocol == NULL) {
		return -EINVAL;
	}
	interface->protocol = protocol;
	protocol->interface = interface;
	int ret = interface->api->init(interface);
	if (ret) {
		return ret;
	}
	ret = protocol->api->init(protocol);
	if (ret) {
		return ret;
	}
	return 0;
}

#endif
