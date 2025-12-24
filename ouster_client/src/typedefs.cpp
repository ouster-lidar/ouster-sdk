#include "ouster/typedefs.h"

#include <array>

namespace ouster {
namespace sdk {
namespace core {

mat4d mat4d_from_array(
    const std::array<double, mat4d::SizeAtCompileTime>& arr) {
    mat4d output;
    for (int i = 0; i < mat4d::RowsAtCompileTime; i++) {
        for (int j = 0; j < mat4d::ColsAtCompileTime; j++) {
            // We know the array is size 16, so this is safe.
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            output(i, j) = arr[(i * mat4d::RowsAtCompileTime) + j];
        }
    }
    return output;
}

std::array<double, mat4d::SizeAtCompileTime> mat4d_to_array(const mat4d& mat) {
    std::array<double, mat4d::SizeAtCompileTime> arr{};
    for (int i = 0; i < mat4d::RowsAtCompileTime; i++) {
        for (int j = 0; j < mat4d::RowsAtCompileTime; j++) {
            // We know the array is size 16, so this is safe.
            // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
            arr[(i * mat4d::RowsAtCompileTime) + j] = mat(i, j);
        }
    }
    return arr;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
