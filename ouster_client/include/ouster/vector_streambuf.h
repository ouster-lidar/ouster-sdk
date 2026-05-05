/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <cstdint>
#include <cstdio>
#include <ios>
#include <iosfwd>
#include <streambuf>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

/**
 * A stream buffer that reads from and writes to a std::vector<uint8_t>.
 *
 * This class can be used to create std::istream and std::ostream objects
 * that operate on a vector of bytes. It supports reading, writing (appending
 * only), and seeking in read mode.
 *
 * The constructor accepts a pointer to a std::vector<uint8_t> to signify that
 * it does not copy or take ownership of the buffer. The caller must ensure that
 * the vector remains valid for the lifetime of the VectorStreamBuf object.
 */
class VectorStreamBuf : public std::streambuf {
   public:
    using ByteVector =
        std::vector<uint8_t>;  ///< An alias for uint8_t vector type

    /**
     * Constructs a VectorStreamBuf that reads from or writes to the given
     * buffer.
     * @param[in] buffer A pointer to a std::vector<uint8_t> to use as the
     * buffer. Must not be null and must remain valid for the lifetime of this
     * object.
     * @throws std::invalid_argument if buffer is null.
     */
    explicit VectorStreamBuf(const ByteVector* buffer);

   protected:
    /**
     * Override the overflow function to write one character at a time.
     * @param[in] character The character to write, or EOF.
     * @return The written character.
     */
    int_type overflow(int_type character) override;

    /**
     * Override the xsputn function to write multiple characters at once.
     * @param[in] source The source buffer to write from.
     * @param[in] count The number of characters to write.
     * @return The number of characters written.
     */
    std::streamsize xsputn(const char* source, std::streamsize count) override;

    /**
     * Override the seekoff function to support seeking in read mode.
     * @param[in] off The offset to seek to.
     * @param[in] way The direction to seek from (beg, cur, end).
     * @param[in] openmode The open mode (in, out).
     * @return The new position, or -1 on error.
     */
    std::streampos seekoff(std::streamoff off, std::ios_base::seekdir way,
                           std::ios_base::openmode openmode) override;

    /**
     * Override the seekpos function to support seeking in read mode.
     * @param[in] position The position to seek to.
     * @param[in] openmode The open mode (in, out).
     * @return The new position, or -1 on error.
     */
    std::streampos seekpos(std::streampos position,
                           std::ios_base::openmode openmode) override;

   private:
    ByteVector* buffer_;  ///< Pointer to the vector buffer (not owned)
};
}  // namespace core
}  // namespace sdk
}  // namespace ouster
