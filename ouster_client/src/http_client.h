/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file http_client.h
 * @brief This file holds a minimal abstract interface for an http client.
 *
 *
 */

#pragma once

#include <algorithm>
#include <string>

namespace ouster {
namespace util {

/**
 * An abstraction of http client to handle http requests
 */
class HttpClient {
   public:
    /**
     * Constructs an HttpClient object to communicate with an http server.
     *
     * @param[in] base_url_ url to the http server.
     */
    HttpClient(const std::string& base_url_) {
        // Properly escape URLs that look like IPv6 addresses (2 or more :'s)
        if (std::count(base_url_.begin(), base_url_.end(), ':') >= 2) {
            base_url = "[" + base_url_ + "]";
        } else {
            base_url = base_url_;
        }
    }

    virtual ~HttpClient() {}

    /**
     * Executes a GET request towards the provided url.
     *
     * @param[in] url http request url.
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return the result of the execution. If request fails it returns an empty
     * string.
     */
    virtual std::string get(const std::string& url, int timeout_sec) const = 0;

    /**
     * Executes a Delete request towards the provided url.
     *
     * @param[in] url http request url.
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return the result of the execution. If request fails it returns an empty
     * string.
     */
    virtual std::string del(const std::string& url, int timeout_sec) const = 0;

    /**
     * Executes a Delete request towards the provided url.
     *
     * @param[in] url http request url.
     * @param[in] data http post data.
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return the result of the execution. If request fails it returns an empty
     * string.
     */
    virtual std::string post(const std::string& url, const std::string& data,
                             int timeout_sec) const = 0;

    /**
     * Executes a Delete request towards the provided url.
     *
     * @param[in] url http request url.
     * @param[in] data http post data.
     * @param[in] timeout_sec The timeout for the request in seconds.
     *
     * @return the result of the execution. If request fails it returns an empty
     * string.
     */
    virtual std::string put(const std::string& url, const std::string& data,
                            int timeout_sec) const = 0;

    /**
     * Encodes the given string as a url.
     *
     * @param[in] str the string to be encoded as a url.
     *
     * @return Returns the string as encoded url.
     */
    virtual std::string encode(const std::string& str) const = 0;

   protected:
    std::string base_url;
};

}  // namespace util
}  // namespace ouster
