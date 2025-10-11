#ifndef ARES_PROTOCOL_H
#define ARES_PROTOCOL_H

#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>

#ifdef __cplusplus
extern "C" {
#endif

enum AresProtocolEvent {
	ARES_PROTOCOL_EVENT_CONNECTED,
	ARES_PROTOCOL_EVENT_DISCONNECTED,
};

struct AresProtocol;

/**
 * @brief API that a protocol must implement.
 *
 * This structure defines the set of functions that a specific protocol
 * (e.g., MAVLink, CANopen) must provide. These functions are called by the
 * interface layer to process data and manage the protocol's lifecycle.
 */
struct AresProtocolAPI {
	void (*handle)(struct AresProtocol *protocol, struct net_buf *buf);
	void (*handle_byte)(struct AresProtocol *protocol, uint8_t byte);
	void (*event)(struct AresProtocol *protocol, enum AresProtocolEvent event);
	int (*init)(struct AresProtocol *protocol);
};

/**
 * @brief Represents a stateless definition of a communication protocol.
 */
struct AresProtocol {
	const char *name;
	const struct AresProtocolAPI *api;
	struct AresInterface *interface;

	void *priv_data;
};

#ifdef __cplusplus
}
#endif

#endif
