#include "ouster/core/camera_model.h"

#include <sstream>
#include <stdexcept>

namespace ouster {
namespace sdk {
namespace core {

CameraModel::CameraModel(float focal_x, float focal_y, float center_x, float center_y)
    : fx(focal_x), fy(focal_y), cx(center_x), cy(center_y) {
    if (!(focal_x > 0.0f) || !(focal_y > 0.0f)) {
        std::stringstream error_stream;
        error_stream << "camera focal length must be greater than zero, got fx=" << focal_x
                     << " fy=" << focal_y;
        throw std::invalid_argument(error_stream.str());
    }
}

bool operator==(const CameraModel& lhs, const CameraModel& rhs) {
    return lhs.fx == rhs.fx && lhs.fy == rhs.fy && lhs.cx == rhs.cx && lhs.cy == rhs.cy;
}

bool operator!=(const CameraModel& lhs, const CameraModel& rhs) {
    return !(lhs == rhs);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
