#pragma once
#include <cstdint>
#include <string>

#include "ouster/osf/file.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * @brief Memory-mapped OSF file implementation.
 *
 * This class provides an interface to read from an OSF file using memory-mapped
 * I/O, allowing efficient access to the file's contents without loading the
 * entire file into memory.
 */
class OUSTER_API_CLASS MemoryMappedOsfFile : public OsfFile {
   public:
    /**
     * @brief Construct a memory-mapped view of the given OSF file.
     *
     * @param[in] filename Path to the OSF file to be memory-mapped.
     */
    OUSTER_API_FUNCTION
    MemoryMappedOsfFile(const std::string& filename);
    OUSTER_API_FUNCTION
    ~MemoryMappedOsfFile() override;
    /**
     * Un-maps memory and essentially invalidates the
     * memory addresses that might be captured within MessageRefs
     * and Reader.
     * @sa ouster::sdk::osf::MessageRef, ouster::sdk::osf::Reader
     */
    OUSTER_API_FUNCTION
    void close() override;

    /**
     * @brief Move constructor.
     */
    OUSTER_API_FUNCTION
    MemoryMappedOsfFile(MemoryMappedOsfFile&&) noexcept;
    OUSTER_API_FUNCTION
    MemoryMappedOsfFile& operator=(MemoryMappedOsfFile&&) noexcept;

    /**
     * @brief Read a buffer from the OSF file at a given offset.
     *
     * Reads a contiguous block of data starting at the specified offset within
     * the OSF file.
     *
     * @param[in] offset Byte offset from the beginning of the file.
     * @return An OsfBuffer containing the requested data.
     *
     * @throws std::out_of_range if the offset exceeds file size or is invalid.
     */
    OUSTER_API_FUNCTION
    OsfBuffer read(OsfOffset offset) override;

    /**
     * @brief Read a buffer from the OSF file relative to a base offset.
     *
     * Computes the absolute position as `base_offset + offset` and reads data
     * starting from that location.
     *
     * @param[in] base_offset Starting reference offset within the file.
     * @param[in] offset Relative offset from the base offset.
     * @return An OsfBuffer containing the requested data.
     *
     * @throws std::out_of_range if the offset exceeds file size or is invalid.
     */
    OUSTER_API_FUNCTION
    OsfBuffer read(OsfOffset base_offset, OsfOffset offset) override;

    /**
     * Returns the size of the OSF file.
     *
     * @return The size of the OSF file in bytes.
     */
    OUSTER_API_FUNCTION
    uint64_t size() const override { return size_; }

   private:
    OUSTER_API_FUNCTION
    uint8_t* buf_raw_ptr(OsfOffset& base_offset, OsfOffset& offset);
    /**
     * Mmaped memory address pointed to the beginning of the file (byte 0)
     */
    uint8_t* file_buf_{nullptr};
    uint64_t size_{0};
    /**
     * Mmaped view handle to keep while the memmap is open on Windows
     * This is not used on Linux, where the mmap is done with a file descriptor
     * and the file descriptor is closed after the mmap is done.`
     */
    uintptr_t memmap_handle_{0};
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
