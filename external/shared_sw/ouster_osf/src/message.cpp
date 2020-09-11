#include "ouster/osf/message.h"

#include "ouster/lidar_scan.h"

#include "png_tools.h"
#include "util_impl.h"
#include "osfChunk_generated.h"

namespace ouster {
namespace OSF {

uint8_t MessageRef::id() const {
    const StampedMessage* sm = reinterpret_cast<const StampedMessage*>(buf_);
    return sm->id();
}

MessageRef::ts_t MessageRef::ts() const {
    const StampedMessage* sm = reinterpret_cast<const StampedMessage*>(buf_);
    return ts_t(sm->ts());
}

MessageType MessageRef::type() const {
    const StampedMessage* sm = reinterpret_cast<const StampedMessage*>(buf_);
    return static_cast<MessageType>(sm->message_type());
}

std::unique_ptr<LidarScan> MessageRef::as_lidar_scan() const {
    const StampedMessage* sm = reinterpret_cast<const StampedMessage*>(buf_);
    if (sm->message_type() != Message_lidar_scan) return nullptr;
    return lidar_scan_from_osf_message(sm, file_info_);
}

std::unique_ptr<Gps> MessageRef::as_gps() const {
    const StampedMessage* sm = reinterpret_cast<const StampedMessage*>(buf_);
    if (sm->message_type() != Message_gps_waypoint) return nullptr;
    return gps_from_osf_message(sm);
}

std::unique_ptr<Imu> MessageRef::as_imu() const {
    const StampedMessage* sm = reinterpret_cast<const StampedMessage*>(buf_);
    return imu_from_osf_message(sm);
}

std::unique_ptr<Trajectory> MessageRef::as_trajectory() const {
    const StampedMessage* sm = reinterpret_cast<const StampedMessage*>(buf_);
    return traj_from_osf_message(sm);
}

std::string to_string(const MessageType message_type) {
    // MessageType reflect the OSF::Message so we are using it's facilities
    const size_t index = static_cast<size_t>(message_type);
    return EnumNamesMessage()[index];
}

}  // namespace OSF
}  // namespace ouster
