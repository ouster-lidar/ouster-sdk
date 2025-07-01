#include <curl/curl.h>

#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

#include "http_client.h"
#include "ouster/impl/logging.h"

class CurlClient : public ouster::util::HttpClient {
    enum class RequestType {
        TYPE_GET = 0,
        TYPE_DELETE = 1,
        TYPE_POST = 2,
        TYPE_PUT = 3
    };

    mutable std::mutex mutex_;

   public:
    explicit CurlClient(const std::string& base_url) : HttpClient(base_url) {
        curl_global_init(CURL_GLOBAL_ALL);
        // NOLINTNEXTLINE(cppcoreguidelines-prefer-member-initializer)
        curl_handle_ = curl_easy_init();
        curl_easy_setopt(curl_handle_, CURLOPT_WRITEFUNCTION,
                         &CurlClient::write_memory_callback);
        curl_easy_setopt(curl_handle_, CURLOPT_WRITEDATA, this);
    }

    // Copy Constructor
    CurlClient(const CurlClient& other) = delete;

    // Copy Assignment Operator
    CurlClient& operator=(const CurlClient& other) = delete;

    // Move Constructor
    CurlClient(CurlClient&& other) noexcept = delete;

    // Move Assignment Operator
    CurlClient& operator=(CurlClient&& other) noexcept = delete;

    // Destructor
    ~CurlClient() override {
        curl_easy_cleanup(curl_handle_);
        curl_global_cleanup();
    }

    std::string get(const std::string& url,
                    int timeout_seconds) const override {
        auto full_url = url_combine(base_url_, url);
        return execute_request(RequestType::TYPE_GET, full_url,
                               timeout_seconds);
    }

    std::string del(const std::string& url,
                    int timeout_seconds) const override {
        auto full_url = url_combine(base_url_, url);
        return execute_request(RequestType::TYPE_DELETE, full_url,
                               timeout_seconds);
    }

    std::string post(const std::string& url, const std::string& data,
                     int timeout_seconds) const override {
        auto full_url = url_combine(base_url_, url);
        return execute_request(RequestType::TYPE_POST, full_url,
                               timeout_seconds, data.c_str());
    }

    std::string put(const std::string& url, const std::string& data,
                    int timeout_seconds) const override {
        auto full_url = url_combine(base_url_, url);
        return execute_request(RequestType::TYPE_PUT, full_url, timeout_seconds,
                               data.c_str());
    }

    std::string encode(const std::string& str) const override {
        auto curl_str_deleter = [](char* str) { curl_free(str); };
        auto encoded_str = std::unique_ptr<char, decltype(curl_str_deleter)>(
            curl_easy_escape(curl_handle_, str.c_str(),
                             static_cast<int>(str.length())),
            curl_str_deleter);
        return std::string{encoded_str.get()};
    }

   private:
    static std::string url_combine(const std::string& url1,
                                   const std::string& url2) {
        if (!url1.empty() && !url2.empty()) {
            if (url1[url1.length() - 1] == '/' && url2[0] == '/') {
                return url1 + url2.substr(1);
            }
            if (url1[url1.length() - 1] != '/' && url2[0] != '/') {
                return url1 + '/' + url2;
            }
        }

        return url1 + url2;
    }

    std::string execute_request(RequestType type, const std::string& url,
                                int timeout_seconds, const char* data = nullptr,
                                int attempts = 3,
                                int retry_delay_ms = 500) const {
        long http_code = 0;  // NOLINT(google-runtime-int); curl docs uses long
        struct curl_slist* hs = nullptr;
        if (attempts < 1) {
            throw std::invalid_argument(
                "CurlClient::execute_request: invalid number of retries");
        }
        if (retry_delay_ms < 0) {
            throw std::invalid_argument(
                "CurlClient::execute_request: invalid retry delay");
        }
        std::lock_guard<std::mutex> guard(mutex_);
        curl_easy_setopt(curl_handle_, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl_handle_, CURLOPT_DEFAULT_PROTOCOL, "http");
        curl_easy_setopt(curl_handle_, CURLOPT_TIMEOUT, timeout_seconds);
        if (type == RequestType::TYPE_GET) {
            curl_easy_setopt(curl_handle_, CURLOPT_CUSTOMREQUEST, 0);
            curl_easy_setopt(curl_handle_, CURLOPT_HTTPGET, 1L);
            curl_easy_setopt(curl_handle_, CURLOPT_HTTPHEADER, 0);
        } else if (type == RequestType::TYPE_DELETE) {
            curl_easy_setopt(curl_handle_, CURLOPT_CUSTOMREQUEST, "DELETE");
            curl_easy_setopt(curl_handle_, CURLOPT_HTTPHEADER, 0);
        } else if (type == RequestType::TYPE_POST) {
            curl_easy_setopt(curl_handle_, CURLOPT_CUSTOMREQUEST, 0);
            hs = curl_slist_append(hs, "Content-Type: application/json");
            curl_easy_setopt(curl_handle_, CURLOPT_POSTFIELDS, data);
        } else if (type == RequestType::TYPE_PUT) {
            curl_easy_setopt(curl_handle_, CURLOPT_CUSTOMREQUEST, "PUT");
            hs = curl_slist_append(hs, "Content-Type: application/json");
            curl_easy_setopt(curl_handle_, CURLOPT_POSTFIELDS, data);
        }
        for (const auto& header : additional_headers_) {
            hs = curl_slist_append(hs, header.c_str());
        }
        if (hs) {
            curl_easy_setopt(curl_handle_, CURLOPT_HTTPHEADER, hs);
        }
        while (attempts-- != 0) {
            buffer_.clear();
            auto res = curl_easy_perform(curl_handle_);
            if (res == CURLE_SEND_ERROR) {
                // Specific versions of curl does't play well with the sensor
                // http server. When CURLE_SEND_ERROR happens for the first time
                // silently re-attempting the http request resolves the problem.
                res = curl_easy_perform(curl_handle_);
            }
            if (res != CURLE_OK) {
                curl_slist_free_all(hs);
                throw std::runtime_error(
                    "CurlClient::execute_request failed for the url: [" + url +
                    "] with the error message: " + curl_easy_strerror(res));
            }
            curl_easy_getinfo(curl_handle_, CURLINFO_RESPONSE_CODE, &http_code);
            if (200 <= http_code && http_code < 300) {
                // HTTP 2XX means a successful response
                curl_slist_free_all(hs);
                return buffer_;
            }
            if (attempts != 0 && 500 <= http_code && http_code < 600) {
                // HTTP 5XX means a server error, so we should re-attempt.
                // log a warning and sleep before re-attempting
                ouster::sensor::logger().warn(
                    "Re-attempting CurlClient::execute_get after failure for "
                    "url: [{}] with the code: [{}] - and return: {}",
                    url, http_code, buffer_);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(retry_delay_ms));
            }
        }

        // the request failed after repeated attempts with a response code other
        // than HTTP 2XX
        curl_slist_free_all(hs);
        throw std::runtime_error(
            std::string("CurlClient::execute_request failed for url: [" + url +
                        "] with the code: [" + std::to_string(http_code) +
                        std::string("] - and return: ") + buffer_));
    }

    static size_t write_memory_callback(void* contents, size_t element_size,
                                        size_t elements_count,
                                        void* user_pointer) {
        size_t size_increment = element_size * elements_count;
        auto* client = static_cast<CurlClient*>(user_pointer);
        auto size_current = client->buffer_.size();
        client->buffer_.resize(size_current + size_increment);
        memcpy(&client->buffer_[size_current], contents, size_increment);
        return size_increment;
    }

    CURL* curl_handle_;
    mutable std::string buffer_;
};
