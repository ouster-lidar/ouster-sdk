#pragma once

#include <spdlog/logger.h>

#include "ouster/ouster_client_export.h"

namespace ouster {
namespace sensor {
namespace impl {

class OUSTER_CLIENT_EXPORT Logger {
   public:
    static Logger& instance();

    spdlog::logger& get_logger();

    void configure_generic_sink(spdlog::sink_ptr sink,
                                const std::string& log_level);
    bool configure_stdout_sink(const std::string& log_level);

    bool configure_file_sink(const std::string& log_level,
                             const std::string& log_file_path, bool rotating,
                             int max_size_in_bytes, int max_files);

   private:
    Logger();

    static const std::string logger_name;
    std::unique_ptr<spdlog::logger> logger_;
};
}  // namespace impl

OUSTER_CLIENT_EXPORT spdlog::logger& logger();

}  // namespace sensor
}  // namespace ouster
