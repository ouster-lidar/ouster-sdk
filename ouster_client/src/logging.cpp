#include "ouster/impl/logging.h"

#include <spdlog/fmt/bundled/args.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <chrono>
#include <iostream>
#include <string>

using namespace ouster::sensor::impl;

const std::string Logger::logger_name{"ouster::sensor"};
const std::string Logger::logger_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");

struct Logger::internal_logger {
    friend Logger;

    internal_logger(const std::string& logger_name, const std::string& pattern)
        : logger_(std::make_unique<spdlog::logger>(
              logger_name, std::make_shared<spdlog::sinks::stdout_sink_st>())),
          pattern_(pattern) {}

    void configure_generic_sink(spdlog::sink_ptr sink,
                                const std::string& log_level) {
        // replace the logger sink with the new sink
        logger_->sinks() = {sink};
        logger_->set_pattern(pattern_);
        auto ll = spdlog::level::from_str(log_level);
        logger_->set_level(ll);
        logger_->flush_on(ll);
    }

    void disable_auto_newline() {
        // TODO: consider using an multi-threaded sinks
        // and add lock guards around the logger usage.
        auto f = std::make_unique<spdlog::pattern_formatter>(
            "%v", spdlog::pattern_time_type::local, std::string(""));
        logger_->set_formatter(std::move(f));
    }

    void enable_auto_newline() { logger_->set_pattern(pattern_); }

    ~internal_logger() {}

   private:
    std::unique_ptr<spdlog::logger> logger_;
    std::string pattern_;
};

Logger& Logger::instance() {
    static Logger logger_singleton;
    return logger_singleton;
}

Logger::Logger()
    : internal_logger_(std::make_unique<Logger::internal_logger>(
          logger_name, logger_pattern)) {
    internal_logger_->logger_->set_level(spdlog::level::info);
    internal_logger_->logger_->flush_on(spdlog::level::info);
}

bool Logger::configure_stdout_sink(const std::string& log_level) {
    try {
        internal_logger_->configure_generic_sink(
            std::make_shared<spdlog::sinks::stdout_sink_st>(), log_level);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << logger_name << " init_logger failed: " << ex.what()
                  << std::endl;
        return false;
    }

    return true;
}

bool Logger::configure_file_sink(const std::string& log_level,
                                 const std::string& log_file_path,
                                 bool rotating, int max_size_in_bytes,
                                 int max_files) {
    try {
        std::shared_ptr<spdlog::sinks::base_sink<spdlog::details::null_mutex>>
            file_sink;
        if (rotating) {
            file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_st>(
                log_file_path, max_size_in_bytes, max_files);
        } else {
            file_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>(
                log_file_path, true);
        }
        internal_logger_->configure_generic_sink(file_sink, log_level);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << logger_name << " init_logger failed: " << ex.what()
                  << std::endl;
        return false;
    }

    return true;
}

std::shared_ptr<Logger::format_builder> Logger::make_builder(
    const std::string& format_string) {
    return std::make_shared<Logger::format_builder>(format_string);
}

struct Logger::format_builder {
    ~format_builder() = default;
    format_builder(const std::string& format_string)
        : format_string(format_string){};

    const std::string& format_string;
    fmt::dynamic_format_arg_store<fmt::format_context> store;
};

Logger::~Logger() {}

std::string Logger::finalize_format_builder(
    std::shared_ptr<Logger::format_builder> builder) {
    return fmt::vformat(builder->format_string, builder->store);
}

void Logger::log(LOG_LEVEL level, const std::string& msg) {
    spdlog::level::level_enum level_out = spdlog::level::debug;

    switch (level) {
        case Logger::LOG_LEVEL::LOG_TRACE:
            level_out = spdlog::level::trace;
            break;
        case Logger::LOG_LEVEL::LOG_DEBUG:
            level_out = spdlog::level::debug;
            break;
        case Logger::LOG_LEVEL::LOG_INFO:
            level_out = spdlog::level::info;
            break;
        case Logger::LOG_LEVEL::LOG_WARN:
            level_out = spdlog::level::warn;
            break;
        case Logger::LOG_LEVEL::LOG_ERROR:
            level_out = spdlog::level::err;
            break;
        case Logger::LOG_LEVEL::LOG_CRITICAL:
            level_out = spdlog::level::critical;
            break;
    };

    internal_logger_->logger_->log(level_out, msg);
}

void Logger::disable_auto_newline() {
    internal_logger_->disable_auto_newline();
}

void Logger::enable_auto_newline() { internal_logger_->enable_auto_newline(); }

#define LOGGER_PROCESS_ARG(SINGLE_LOGGER_PROCESS_ARG_type) \
    template void Logger::process_arg(                     \
        std::shared_ptr<Logger::format_builder> builder,   \
        SINGLE_LOGGER_PROCESS_ARG_type data);

LOGGER_PROCESS_ARG(std::string);
LOGGER_PROCESS_ARG(std::string&);
LOGGER_PROCESS_ARG(const std::string&);

LOGGER_PROCESS_ARG(char*);
LOGGER_PROCESS_ARG(const char*);

LOGGER_PROCESS_ARG(int8_t);
LOGGER_PROCESS_ARG(int16_t);
LOGGER_PROCESS_ARG(int32_t);
LOGGER_PROCESS_ARG(int64_t);
#if defined(__EMSCRIPTEN__) || defined(__APPLE__) || defined(WIN32)
LOGGER_PROCESS_ARG(long);
#endif

LOGGER_PROCESS_ARG(uint8_t);
LOGGER_PROCESS_ARG(uint16_t);
LOGGER_PROCESS_ARG(uint32_t);
LOGGER_PROCESS_ARG(uint64_t);
#if defined(__EMSCRIPTEN__) || defined(__APPLE__) || defined(WIN32)
LOGGER_PROCESS_ARG(unsigned long);
#endif

LOGGER_PROCESS_ARG(float);
LOGGER_PROCESS_ARG(double);
LOGGER_PROCESS_ARG(long double);

LOGGER_PROCESS_ARG(bool);

template <typename T>
void Logger::process_arg(std::shared_ptr<Logger::format_builder> builder,
                         T data) {
    builder->store.push_back(data);
}

namespace ouster {
namespace sensor {
ouster::sensor::impl::Logger& logger() {
    return ouster::sensor::impl::Logger::instance();
}
}  // namespace sensor
}  // namespace ouster
