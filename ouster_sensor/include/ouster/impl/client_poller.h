/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <memory>

#include "ouster/client.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace sensor {
namespace impl {

/**
 * Poller used in multiclient scenarios
 */
struct OUSTER_API_CLASS ClientPoller;

/**
 * produces uninitialized poller
 */
OUSTER_API_FUNCTION
std::shared_ptr<ClientPoller> make_poller();

/**
 * Reset poller. Must be called prior to any other operations
 *
 * @param[in] poller ClientPoller to reset
 */
OUSTER_API_FUNCTION
void reset_poll(ClientPoller& poller);

/**
 * Set poller to watch client on the next poll call
 *
 * @param[in] poller ClientPoller
 * @param[in] cli client to watch
 */
OUSTER_API_FUNCTION
void set_poll(ClientPoller& poller, const Client& cli);

/**
 * Polls clients previously set with `set_poll`
 *
 * @param[in] poller ClientPoller
 * @param[in] timeout_sec timeout in seconds
 *
 * @return -1 for error, 0 for timeout, otherwise number of messages received
 */
OUSTER_API_FUNCTION
int poll(ClientPoller& poller, int timeout_sec = 1);

/**
 * Retrieves error state of the poller
 *
 * @param[in] poller ClientPoller
 *
 * @return client_state which is one of ERR or EXIT on error,
 *         otherwise returning TIMEOUT if no error occurred
 */
OUSTER_API_FUNCTION
ClientState get_error(const ClientPoller& poller);

/**
 * Retrieve poll results for particular client
 *
 * @param[in] poller ClientPoller
 * @param[in] cli client to retrieve results for
 *
 * @return client_state comprising of either LIDAR_DATA or IMU_DATA, or TIMEOUT
 *         if no data was received
 */
OUSTER_API_FUNCTION
ClientState get_poll(const ClientPoller& poller, const Client& cli);

}  // namespace impl
}  // namespace sensor
}  // namespace sdk
}  // namespace ouster
