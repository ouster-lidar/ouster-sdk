#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ouster/core/visibility.h"
#include "ouster/osf/metadata.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * Represents the metadata entry associated with a SensorInfoStream.
 */
class OUSTER_API_CLASS EmptyMeta : public MetadataEntryHelper<EmptyMeta> {
   public:
    /**
     * Create a EmptyMeta.
     */
    OUSTER_API_FUNCTION
    EmptyMeta();

    /**
     * @copydoc MetadataEntry::buffer
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> buffer() const final;

    /**
     * Create a EmptyMeta object from a byte array.
     *
     * @todo Figure out why this wasnt just done as a constructor overload.
     *
     * @relates MetadataEntry::from_buffer
     *
     * @param[in] buf The raw flatbuffer byte vector to initialize from.
     * @return The new EmptyMeta cast as a MetadataEntry
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<MetadataEntry> from_buffer(const ouster::sdk::osf::OsfBuffer& buf);

    /**
     * Get the string representation for the EmptyMeta object.
     *
     * @relates MetadataEntry::repr
     *
     * @return The string representation for the EmptyMeta
     * object.
     */
    OUSTER_API_FUNCTION
    std::string repr() const override;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
