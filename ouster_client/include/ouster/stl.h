#pragma once

#include <cstdint>
#include <vector>

#include "ouster/mesh.h"
#include "ouster/sha256.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * This class represents a 3D model in STL (stereolithography) file format.
 * It provides functionality to load an STL file from various sources,
 * compute its SHA-256 hash, and convert it to a Mesh object for further
 * processing. The STL file can be in either ASCII or binary format.
 */
class OUSTER_API_CLASS Stl {
   public:
    /**
     * Constructs an Stl object from a file path.
     * @param[in] file_path The path to the STL file.
     * @throws std::runtime_error if the file cannot be read or is not a valid
     * STL file.
     */
    OUSTER_API_FUNCTION
    explicit Stl(const std::string& file_path);

    /**
     * Constructs an Stl object from an input stream.
     * @param[in] file The input stream containing the STL file data.
     * @throws std::runtime_error if the stream cannot be read or is not a valid
     * STL file.
     */
    OUSTER_API_FUNCTION
    explicit Stl(std::istream& file);

    /**
     * Constructs an Stl object from a binary blob.
     * @param[in] blob The binary blob containing the STL file data.
     * @throws std::runtime_error if the blob is not a valid STL file.
     */
    OUSTER_API_FUNCTION
    explicit Stl(std::vector<uint8_t> blob);

    /**
     * Returns the binary blob containing the STL file data.
     * @return A vector of bytes representing the STL file data.
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> blob() const;

    /**
     * Computes the SHA-256 hash of the STL file data.
     * @return The SHA-256 hash of the STL file data.
     */
    OUSTER_API_FUNCTION
    Sha256 hash() const;

    /**
     * Source of the zone origin used for rendering.
     * - BODY: Use the extrinsics matrix as the zone origin
     * - SENSOR: Use the sensor origin as the zone origin
     */
    enum class CoordinateFrame : uint8_t {
        NONE = 0,    ///< No coordinate frame specified
        BODY = 1,    ///< Use the extrinsics matrix as the zone origin
        SENSOR = 2,  ///< Use the sensor origin as the zone origin
    };

    CoordinateFrame coordinate_frame{
        CoordinateFrame::NONE};  ///< The zone origin

    /**
     * Converts an CoordinateFrame enum value to its corresponding string
     * representation.
     * @param[in] str The string to convert.
     * @param[out] coordinate_frame The resulting CoordinateFrame enum value.
     * @return true if the conversion was successful, false otherwise.
     */
    OUSTER_API_FUNCTION
    static bool string_to_coordinate_frame(
        const std::string& str, Stl::CoordinateFrame& coordinate_frame);

    /**
     * Converts the STL file data to a Mesh object.
     * @return A Mesh object representing the STL file data.
     * @throws std::runtime_error if the STL file data cannot be parsed.
     */
    OUSTER_API_FUNCTION
    Mesh to_mesh() const;

    /**
     * Compares two Stl objects for equality.
     * @param[in] lhs The first Stl object.
     * @param[in] rhs The second Stl object.
     * @return true if the two Stl objects are equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    friend bool operator==(const Stl& lhs, const Stl& rhs);

    /**
     * Compares two Stl objects for inequality.
     * @param[in] lhs The first Stl object.
     * @param[in] rhs The second Stl object.
     * @return true if the two Stl objects are not equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    friend bool operator!=(const Stl& lhs, const Stl& rhs);

    std::string filename{};  ///< Original filename of the STL file

   private:
    void load_from_stream(std::istream& stream);
    std::vector<uint8_t> blob_{};  ///< Binary blob containing stl file
};

/**
 * Converts an CoordinateFrame enum value to its corresponding string
 * representation.
 * @param[in] coordinate_frame The CoordinateFrame enum value to convert.
 * @return The string representation of the CoordinateFrame enum value.
 */
OUSTER_API_FUNCTION
std::string to_string(Stl::CoordinateFrame coordinate_frame);

}  // namespace core
}  // namespace sdk
}  // namespace ouster
