/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

namespace ouster {
namespace sensor {
namespace impl {

/**
 * Poller used in multiclient scenarios
 */
struct client_poller;

/**
 * produces uninitialized poller
 */
std::shared_ptr<client_poller> make_poller();

/**
 * Reset poller. Must be called prior to any other operations
 *
 * @param[in] poller client_poller to reset
 */
void reset_poll(client_poller& poller);

/**
 * Set poller to watch client on the next poll call
 *
 * @param[in] poller client_poller
 * @param[in] cli client to watch
 */
void set_poll(client_poller& poller, const client& cli);

/**
 * Polls clients previously set with `set_poll`
 *
 * @param[in] poller client_poller
 * @param[in] timeout_sec timeout in seconds
 *
 * @return -1 for error, 0 for timeout, otherwise number of messages received
 */
int poll(client_poller& poller, int timeout_sec = 1);

/**
 * Retrieves error state of the poller
 *
 * @param[in] poller client_poller
 *
 * @return client_state which is one of CLIENT_ERROR or EXIT on error,
 *         otherwise returning TIMEOUT if no error occurred
 */
client_state get_error(const client_poller& poller);

/**
 * Retrieve poll results for particular client
 *
 * @param[in] poller client_poller
 * @param[in] cli client to retrieve results for
 *
 * @return client_state comprising of either LIDAR_DATA or IMU_DATA, or TIMEOUT
 *         if no data was received
 */
client_state get_poll(const client_poller& poller, const client& cli);

}  // namespace impl
}  // namespace sensor
}  // namespace ouster
