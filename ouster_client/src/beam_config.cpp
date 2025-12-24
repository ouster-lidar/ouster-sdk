#include "ouster/beam_config.h"

#include <utility>
#include <vector>

#include "ouster/typedefs.h"
#include "ouster/xyzlut.h"

namespace ouster {
namespace sdk {
namespace core {

namespace {
mat4d scale_translation(const mat4d& mat) {
    mat4d res = mat;
    res(0, 3) *= 1000;
    res(1, 3) *= 1000;
    res(2, 3) *= 1000;
    return res;
}
}  // namespace

BeamConfig::BeamConfig(uint32_t n_cols_init,
                       const std::vector<double>& px_altitudes_init,
                       const std::vector<double>& px_azimuths_init,
                       mat4d beam_to_lidar_transform_init,
                       mat4d lidar_to_sensor_transform_init,
                       mat4d sensor_to_body_transform_init,
                       float m_per_zmbin_init, uint64_t serial_number_init)
    : n_cols(n_cols_init),
      n_rows(px_altitudes_init.size()),
      beam_to_lidar_transform(std::move(beam_to_lidar_transform_init)),
      lidar_to_sensor_transform(std::move(lidar_to_sensor_transform_init)),
      sensor_to_body_transform(std::move(sensor_to_body_transform_init)),
      m_per_zmbin(m_per_zmbin_init),
      serial_number(serial_number_init),
      px_altitudes(px_altitudes_init),
      px_azimuths(px_azimuths_init),
      lut(std::make_shared<XYZLut>(
          make_xyz_lut(n_cols, n_rows, 0.001, beam_to_lidar_transform,
                       scale_translation(sensor_to_body_transform) *
                           lidar_to_sensor_transform,
                       px_azimuths, px_altitudes))),
      lut_no_sensor_to_body_transform(std::make_shared<XYZLut>(
          make_xyz_lut(n_cols, n_rows, 0.001, beam_to_lidar_transform,
                       lidar_to_sensor_transform, px_azimuths, px_altitudes))) {
    if (beam_to_lidar_transform.isZero()) {
        throw std::logic_error("BeamConfig: beam_to_lidar_transform not set");
    }
    if (lidar_to_sensor_transform.isZero()) {
        throw std::logic_error("BeamConfig: lidar_to_sensor_transform not set");
    }
    if (sensor_to_body_transform.isZero()) {
        throw std::logic_error("BeamConfig: sensor_to_body_transform not set");
    }
}

bool BeamConfig::operator==(BeamConfig const& rhs) const {
    return (n_cols == rhs.n_cols) && (n_rows == rhs.n_rows) &&
           (px_altitudes == rhs.px_altitudes) &&
           (px_azimuths == rhs.px_azimuths) &&
           (beam_to_lidar_transform == rhs.beam_to_lidar_transform) &&
           (lidar_to_sensor_transform == rhs.lidar_to_sensor_transform) &&
           (sensor_to_body_transform == rhs.sensor_to_body_transform) &&
           (m_per_zmbin == rhs.m_per_zmbin) &&
           (serial_number == rhs.serial_number);
}

bool BeamConfig::operator!=(BeamConfig const& rhs) const {
    return !(*this == rhs);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
