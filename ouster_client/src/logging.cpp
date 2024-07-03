#include "ouster/impl/logging.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_sinks.h>
#include <spdlog/spdlog.h>

#include <iostream>

using namespace ouster::sensor::impl;

const std::string Logger::logger_name{"ouster::sensor"};

Logger& Logger::instance() {
    static Logger logger_singleton;
    return logger_singleton;
}

spdlog::logger& Logger::get_logger() { return *logger_; }

Logger::Logger()
    : logger_(std::make_unique<spdlog::logger>(
          logger_name, std::make_shared<spdlog::sinks::stdout_sink_st>())) {
    logger_->set_level(spdlog::level::info);
    logger_->flush_on(spdlog::level::info);
}

void Logger::configure_generic_sink(spdlog::sink_ptr sink,
                                    const std::string& log_level) {
    // replace the logger sink with the new sink
    logger_->sinks() = {sink};

    auto ll = spdlog::level::from_str(log_level);

    logger_->set_level(ll);
    logger_->flush_on(ll);
}

bool Logger::configure_stdout_sink(const std::string& log_level) {
    try {
        configure_generic_sink(
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
        configure_generic_sink(file_sink, log_level);
    } catch (const spdlog::spdlog_ex& ex) {
        std::cerr << logger_name << " init_logger failed: " << ex.what()
                  << std::endl;
        return false;
    }

    return true;
}

namespace ouster {
namespace sensor {
spdlog::logger& logger() { return Logger::instance().get_logger(); }
}  // namespace sensor
}  // namespace ouster
