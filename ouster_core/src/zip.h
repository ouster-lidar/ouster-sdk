#pragma once

#include <zip.h>

#include <cstdint>
#include <string>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

class Zip {
   public:
    Zip();
    explicit Zip(const std::vector<uint8_t>& zip_bytes);
    Zip(const Zip&) = delete;
    Zip& operator=(const Zip&) = delete;
    Zip(Zip&&) = delete;
    Zip& operator=(Zip&&) = delete;
    std::vector<uint8_t> get_file(const std::string& filename);
    std::string get_file_as_string(const std::string& filename);
    void add_file(const std::string& name, const std::string& data);
    void add_file(const std::string& name, const std::vector<uint8_t>& data);
    std::vector<uint8_t> to_bytes();
    ~Zip();

   private:
    Zip(const uint8_t* data, size_t size, int flags);
    zip_t* archive_{nullptr};
    zip_source_t* source_{nullptr};
    zip_error_t error_;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
