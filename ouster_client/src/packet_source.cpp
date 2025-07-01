#include "ouster/packet_source.h"

namespace ouster {
namespace core {
PacketIterator PacketSource::end() const { return PacketIterator(this); }
}  // namespace core
}  // namespace ouster
