#pragma once
#include <cstdint>
#include <memory>
#include <vector>

#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

// TODO[tws] replace "load_data" methods with constructors, remove has_value
/**
 * @brief Buffer class for storing and accessing OSF binary data
 * including header, metadata, message.
 *
 * It supports memory-mapped or internal (owned) data,
 * with interfaces to load, access, and reset buffer contents.
 */
class OUSTER_API_CLASS OsfBuffer {
   public:
    OUSTER_API_FUNCTION
    OsfBuffer();

    OUSTER_API_FUNCTION
    ~OsfBuffer();
    /**
     * Clears the buffer and resets internal state.
     *
     * For owned data, the memory is released. For memory-mapped data,
     * the pointer is cleared but memory is not unmapped or freed.
     */
    OUSTER_API_FUNCTION
    void reset();

    /**
     * @brief Copy constructor
     * @param[in] other The buffer to copy from.
     */
    OUSTER_API_FUNCTION
    OsfBuffer(const OsfBuffer& other);
    OUSTER_API_FUNCTION
    OsfBuffer& operator=(const OsfBuffer& other);

    OUSTER_API_FUNCTION
    bool operator==(const OsfBuffer& other) const;

    // TODO[tws] replace load_data methods with constructors and remove
    // has_value.
    /**
     * @brief Load buffer using externally managed data (memory-mapped).
     *
     * This does not take ownership of the data and assumes the lifetime of the
     * provided memory is externally managed.
     *
     * @param[in] data Pointer to external data buffer.
     * @param[in] size Size of the data in bytes.
     *
     * @throws std::logic_error if data is null.
     */
    OUSTER_API_FUNCTION
    void load_data(const uint8_t* data, uint64_t size);

    /**
     * @brief Load buffer by moving from a provided vector.
     *
     * Takes ownership of the provided vector contents.
     *
     * @param[in] data Rvalue reference to a vector of bytes.
     *
     * @throws std::logic_error if data is empty.
     */
    OUSTER_API_FUNCTION
    void load_data(std::vector<uint8_t>&& data);

    /**
     * @brief Load buffer by copying from a provided vector.
     *
     * Copies the provided vector contents.
     *
     * @param[in] data Rvalue reference to a vector of bytes.
     *
     * @throws std::logic_error if data is empty.
     */
    OUSTER_API_FUNCTION
    void load_data(const std::vector<uint8_t>& data);

    /**
     * @brief Load buffer by referencing a provided base buffer.
     *
     * @param[in] base_buffer The base OsfBuffer to use as the source.
     * @param[in] offset The offset within the base buffer to start.
     * @param[in] size The number of bytes.
     */
    OUSTER_API_FUNCTION
    void load_data(const class OsfBuffer& base_buffer, uint64_t offset,
                   uint64_t size);

    /**
     * @brief Get a pointer to the underlying data.
     *
     * If the buffer was loaded using memory-mapped data, that pointer is
     * returned. Otherwise, the internal owned memory is returned.
     *
     * @return Pointer to the data, or nullptr if uninitialized.
     */
    OUSTER_API_FUNCTION
    const uint8_t* data() const;

    /**
     * @brief Returns a const iterator to the beginning of the buffer.
     * @return A const iterator pointing to the first byte.
     */
    OUSTER_API_FUNCTION
    const uint8_t* cbegin() const;

    /**
     * @brief Returns a const iterator to the end of the buffer.
     * @return A const iterator pointing past the last byte.
     */
    OUSTER_API_FUNCTION
    const uint8_t* cend() const;

    /**
     * @brief Get the size of the buffer in bytes.
     *
     * @return Size of the buffer content.
     */
    OUSTER_API_FUNCTION
    uint64_t size() const;

    /**
     * @brief Check if the buffer contains valid data.
     *
     * @return True if the buffer has been initialized with valid data.
     */
    OUSTER_API_FUNCTION
    bool has_value() const;

   protected:
    const uint8_t* memory_mapped_data_{nullptr};

    // allows sharing the data between instances of OsfBuffer
    std::shared_ptr<std::vector<uint8_t>> non_memory_mapped_data_;
    uint64_t size_{0};
    uint64_t offset_{0};
    bool has_value_{false};
};

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
