#pragma once
#include <cstdint>
#include <fstream>
#include <string>

#include "ouster/osf/file.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * @brief OSF file reader using standard file streams.
 *
 * Provides a stream-based implementation of the `OsfFile` interface,
 * enabling efficient reading of OSF file content without
 * memory-mapping or loading the full file into memory.
 *
 * This implementation uses `std::ifstream` to support seeking and reading
 * arbitrary byte ranges, such as metadata and chunked sensor data.
 */
class OUSTER_API_CLASS StreamOsfFile : public OsfFile {
   public:
    /**
     * @brief Construct a new StreamOsfFile from the given filename.
     *
     * Opens the file for binary input and initializes header and metadata
     * buffers.
     *
     * @param[in] filename Path to the OSF file.
     */
    OUSTER_API_FUNCTION
    StreamOsfFile(const std::string& filename);
    OUSTER_API_FUNCTION
    ~StreamOsfFile() override;
    /**
     * @brief Move constructor.
     *
     * Transfers ownership of file stream and state from another instance.
     */
    OUSTER_API_FUNCTION
    StreamOsfFile(StreamOsfFile&&);
    OUSTER_API_FUNCTION
    StreamOsfFile& operator=(StreamOsfFile&&);
    /**
     * Clears file handle and allocated resources.
     * @sa ouster::osf::MessageRef, ouster::osf::Reader
     */
    OUSTER_API_FUNCTION
    void close() override;

    // TODO[tws] document
    OUSTER_API_FUNCTION
    OsfBuffer read(OsfOffset offset) override;

    // TODO[tws] document
    OUSTER_API_FUNCTION
    OsfBuffer read(OsfOffset base_offset, OsfOffset offset) override;

    /**
     * Returns the size of the OSF file.
     *
     * @return The size of the OSF file in bytes.
     */
    OUSTER_API_FUNCTION
    uint64_t size() const override;

   private:
    /**
     * File stream for reading.
     */
    StreamOsfFile& seek(uint64_t pos);
    std::ifstream file_stream_;

    /// A method for checking errors and raising the appropriate exception
    void check_stream_bits();

    uint64_t size_;
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
