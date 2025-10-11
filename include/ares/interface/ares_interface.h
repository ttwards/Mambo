#ifndef _ARES_INTERFACE_H_
#define _ARES_INTERFACE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <zephyr/net_buf.h>

#ifdef __cplusplus
extern "C" {
#endif

struct AresProtocol;
struct AresInterface;

/**
 * @brief API that an interface must implement.
 *
 * This structure defines the set of functions that a specific interface
 * (e.g., a UART or USB driver) must provide. It is primarily used by the
 * protocol layer to send data.
 */
struct AresInterfaceAPI {
	int (*send)(struct AresInterface *interface, struct net_buf *buf);
	int (*send_with_lock)(struct AresInterface *interface, struct net_buf *buf,
			      struct k_mutex *mutex);
	int (*send_raw)(struct AresInterface *interface, uint8_t *data, uint16_t len);

	int (*connect)(struct AresInterface *interface);
	int (*disconnect)(struct AresInterface *interface);
	bool (*is_connected)(struct AresInterface *interface);

	struct net_buf *(*alloc_buf)(struct AresInterface *interface);
	struct net_buf *(*alloc_buf_with_data)(struct AresInterface *interface, void *data,
					       size_t size);

	int (*init)(struct AresInterface *interface);
};

/**
 * @brief Represents a specific communication interface instance (e.g., a UART port).
 *
 * This structure holds the state of the interface, including the loaded protocol
 * and any driver-specific data.
 */
struct AresInterface {
	const char *name;
	const struct AresInterfaceAPI *api;
	struct AresProtocol *protocol; // The protocol loaded onto this interface

	void *priv_data; // Interface-specific private data (e.g., UART device ptr)
};

#ifdef __cplusplus
}
#endif

#endif
