#include "ouster/core/zone_state.h"

namespace ouster {
namespace sdk {
namespace core {

bool operator==(const ZoneState& lhs, const ZoneState& rhs) {
    return lhs.live == rhs.live && lhs.id == rhs.id && lhs.error_flags == rhs.error_flags &&
           lhs.trigger_type == rhs.trigger_type && lhs.trigger_status == rhs.trigger_status &&
           lhs.triggered_frames == rhs.triggered_frames && lhs.count == rhs.count &&
           lhs.occlusion_count == rhs.occlusion_count && lhs.invalid_count == rhs.invalid_count &&
           lhs.max_count == rhs.max_count && lhs.min_range == rhs.min_range &&
           lhs.max_range == rhs.max_range && lhs.mean_range == rhs.mean_range;
}

bool operator!=(const ZoneState& lhs, const ZoneState& rhs) {
    return !(lhs == rhs);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
