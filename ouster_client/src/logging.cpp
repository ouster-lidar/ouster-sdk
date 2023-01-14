#include "logging.h"

#include <spdlog/spdlog.h>

#if (SPDLOG_VER_MAJOR < 1)
namespace spdlog {
namespace sinks {
// rename simple_file_sink_st to basic_file_sink_st
using basic_file_sink_st = simple_file_sink_st;
}  // namespace sinks
}  // namespace spdlog
#else
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#endif

#include <spdlog/sinks/stdout_sinks.h>

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

void Logger::update_sink_and_log_level(spdlog::sink_ptr sink,
                                       const std::string& log_level) {
#if (SPDLOG_VER_MAJOR < 1)
    // recreate the logger with the new sink
    logger_ = std::make_unique<spdlog::logger>(logger_name, sink);

    using namespace spdlog::level;
    auto idx = std::find(std::begin(level_names), std::end(level_names),
                         log_level.c_str());
    auto ll = idx == std::end(level_names)
                  ? spdlog::level::off
                  : static_cast<level_enum>(
                        std::distance(std::begin(level_names), idx));
#else
    // replace the logger sink with the new sink
    logger_->sinks() = {sink};

    auto ll = spdlog::level::from_str(log_level);
#endif
    logger_->set_level(ll);
    logger_->flush_on(ll);
}

bool Logger::configure_stdout_sink(const std::string& log_level) {
    try {
        update_sink_and_log_level(
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
        update_sink_and_log_level(file_sink, log_level);
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
