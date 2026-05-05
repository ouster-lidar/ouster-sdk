#include "test_utils.h"

#include <fstream>
#include <ios>
#include <limits>
#include <string>
#include <vector>

// TODO[tws] consider using tellg
// TODO[tws] dedupe (compat-ops)
size_t get_file_size(const std::string& path) {
    std::ifstream file(path, std::ios::in | std::ios::binary);
    file.ignore(std::numeric_limits<std::streamsize>::max());
    if (file.bad()) {
        throw std::runtime_error("Error reading " + path);
    }
    return file.gcount();
}

// TODO[tws] dedupe (compat-ops)
std::vector<uint8_t> get_file_as_bytes(const std::string& path) {
    std::vector<uint8_t> bytes(get_file_size(path));
    std::ifstream file(path, std::ios::in | std::ios::binary);
    file.read(reinterpret_cast<char*>(bytes.data()), bytes.size());
    return bytes;
}
