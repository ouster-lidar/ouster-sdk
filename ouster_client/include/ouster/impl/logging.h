/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <memory>
#include <string>

namespace ouster {
namespace sensor {
namespace impl {

class Logger {
   private:
    struct format_builder;
    std::string finalize_format_builder(
        std::shared_ptr<Logger::format_builder>);
    struct internal_logger;

   public:
    Logger();
    ~Logger();

    enum LOG_LEVEL {
        LOG_TRACE,
        LOG_DEBUG,
        LOG_INFO,
        LOG_WARN,
        LOG_ERROR,
        LOG_CRITICAL
    };

    static Logger& instance();

    bool configure_stdout_sink(const std::string& log_level);

    bool configure_file_sink(const std::string& log_level,
                             const std::string& log_file_path, bool rotating,
                             int max_size_in_bytes, int max_files);

    template <typename... Args>
    void trace(const std::string& format_string, Args&&... args) {
        return log(LOG_LEVEL::LOG_TRACE, format_string,
                   std::forward<Args>(args)...);
    };

    template <typename... Args>
    void debug(const std::string& format_string, Args&&... args) {
        return log(LOG_LEVEL::LOG_DEBUG, format_string,
                   std::forward<Args>(args)...);
    };

    template <typename... Args>
    void info(const std::string& format_string, Args&&... args) {
        return log(LOG_LEVEL::LOG_INFO, format_string,
                   std::forward<Args>(args)...);
    };

    template <typename... Args>
    void warn(const std::string& format_string, Args&&... args) {
        return log(LOG_LEVEL::LOG_WARN, format_string,
                   std::forward<Args>(args)...);
    };

    template <typename... Args>
    void error(const std::string& format_string, Args&&... args) {
        return log(LOG_LEVEL::LOG_ERROR, format_string,
                   std::forward<Args>(args)...);
    };

    template <typename... Args>
    void critical(const std::string& format_string, Args&&... args) {
        return log(LOG_LEVEL::LOG_CRITICAL, format_string,
                   std::forward<Args>(args)...);
    };

    template <typename... Args>
    void log(LOG_LEVEL level, const std::string& format_string,
             Args&&... args) {
        std::shared_ptr<Logger::format_builder> builder =
            make_builder(format_string);
        process_args(builder, std::forward<Args>(args)...);
        return log(level, finalize_format_builder(builder));
    }

    void log(LOG_LEVEL level, const std::string& msg);

    void disable_auto_newline();

    void enable_auto_newline();

   private:
    // Base Case
    template <typename T>
    void process_args(std::shared_ptr<Logger::format_builder> builder,
                      T next_arg) {
        process_arg(builder, next_arg);
    }

    // Recursive Case
    template <typename T, typename... Args>
    void process_args(std::shared_ptr<Logger::format_builder> builder,
                      T next_arg, Args&&... args) {
        process_arg(builder, next_arg);
        process_args(builder, std::forward<Args>(args)...);
    }

    template <typename T>
    void process_arg(std::shared_ptr<Logger::format_builder> builder, T data);

    std::shared_ptr<Logger::format_builder> make_builder(
        const std::string& format_string);

    static const std::string logger_name;
    static const std::string logger_pattern;
    std::unique_ptr<internal_logger> internal_logger_;
};
}  // namespace impl

ouster::sensor::impl::Logger& logger();

}  // namespace sensor
}  // namespace ouster
