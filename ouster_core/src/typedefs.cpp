#include "ouster/core/typedefs.h"

#include <algorithm>
#include <array>

namespace ouster {
namespace sdk {
namespace core {

mat4d mat4d_from_array(const std::array<double, mat4d::SizeAtCompileTime>& arr) {
    mat4d output;
    std::copy(arr.begin(), arr.end(), output.data());
    return output;
}

std::array<double, mat4d::SizeAtCompileTime> mat4d_to_array(const mat4d& mat) {
    std::array<double, mat4d::SizeAtCompileTime> arr{};
    std::copy(mat.data(), mat.data() + 16, arr.data());
    return arr;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
