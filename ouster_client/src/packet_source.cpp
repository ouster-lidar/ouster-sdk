#include "ouster/packet_source.h"

namespace ouster {
namespace sdk {
namespace core {
PacketIterator PacketSource::end() const { return PacketIterator(this); }
}  // namespace core
}  // namespace sdk
}  // namespace ouster
