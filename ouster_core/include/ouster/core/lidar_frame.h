/**
 * Copyright (c) 2018, Ouster, Inc.  All rights reserved.  @file
 * @brief Holds lidar data by field in row-major order
 */

#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/core/defaults.h"
#include "ouster/core/deprecation.h"
#include "ouster/core/field.h"
#include "ouster/core/object.h"
#include "ouster/core/packet.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"
#include "ouster/core/zone_state.h"

namespace ouster {
namespace sdk {
namespace core {
// Forward declaration
class SensorInfo;

/**
 * Description for a field that we desire in a lidar frame
 */
struct OUSTER_API_CLASS FieldType {
    std::string name;                                  ///< Name of the field
    ChanFieldType element_type;                        ///< Type of field elements
    std::vector<size_t> extra_dims;                    ///< Additional dimensions of the field
    FieldClass field_class = FieldClass::PIXEL_FIELD;  ///< Category of field, determines the
                                                       ///< first dimensions of the field

    /**
     * Initialize a default FieldType with no name.
     */
    OUSTER_API_FUNCTION
    FieldType();

    /**
     * Initialize a lidar frame from another with only the indicated fields.
     * Casts, zero pads or removes fields from the original frame if necessary.
     *
     * @param[in] name_ Name of the field described by the type.
     * @param[in] element_type_ Primitive type for elements in the field.
     * @param[in] extra_dims_ Additional dimensions of the field
     * @param[in] class_ Category of field, determins the first dimensions of
                         the field
     */
    OUSTER_API_FUNCTION
    FieldType(std::string name_, ChanFieldType element_type_, std::vector<size_t> extra_dims_ = {},
              FieldClass class_ = FieldClass::PIXEL_FIELD);

    /**
     * Less than operated needed to allow sorting FieldTypes by name
     *
     * @param[in] other The FieldType to compare with
     *
     * @return if the name is "less than" the provided FieldType
     */
    OUSTER_API_FUNCTION
    inline bool operator<(const FieldType& other) const {
        return name < other.name;
    }
};

/**
 * Get string representation of a FieldType.
 *
 * @param[in] field_type The field type to get the string representation of.
 *
 * @return string representation of the FieldType.
 */
OUSTER_API_FUNCTION
std::string to_string(const FieldType& field_type);

/**
 * Equality for FieldTypes.
 *
 * @param[in] a The first type to compare.
 * @param[in] b The second type to compare.
 *
 * @return if a == b.
 */
OUSTER_API_FUNCTION
bool operator==(const FieldType& a, const FieldType& b);

/**
 * Equality for FieldTypes.
 *
 * @param[in] a The first type to compare.
 * @param[in] b The second type to compare.
 *
 * @return if a != b.
 */
OUSTER_API_FUNCTION
bool operator!=(const FieldType& a, const FieldType& b);

/**
 * Alias for the lidar frame field types
 */
using LidarFrameFieldTypes = std::vector<FieldType>;

/**
 * Data structure for efficient operations on aggregated lidar data.
 *
 * Stores each field (range, intensity, etc.) contiguously as a H x W block of
 * 4-byte unsigned integers, where H is the number of beams and W is the
 * horizontal resolution (e.g. 512, 1024, 2048).
 *
 * Note: this is the "staggered" representation where each column corresponds
 * to a single measurement in time. Use the destagger() function to create an
 * image where columns correspond to a single azimuth angle.
 */
class OUSTER_API_CLASS LidarFrame {
   public:
    template <typename T>
    using Header = Eigen::Array<T, Eigen::Dynamic, 1>;  ///< Header typedef

   private:
    std::unordered_map<std::string, Field> fields_;
    std::unordered_map<std::string, std::vector<Object>> objects_;

    // Required special case "fields"
    Field timestamp_;
    Field measurement_id_;
    Field status_;
    Field packet_timestamp_;
    Field body_to_world_;

    /**
     * The alert flags field, from eUDP packet headers.
     */
    Field alert_flags_;

    /**
     * The number of packets used to produce a full frame given the width in
     * pixels and the number of columns per packet.
     */
    size_t packet_count_{0};

   public:
    /**
     * Frame horizontal resolution.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    size_t w{0};

    /**
     * Frame vertical resolution.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    size_t h{0};

    /**
     * Frame status - information from the packet header which corresponds to a
     * frame
     *
     * @warning Member variables: use with caution, some of these will become
     * private.
     */
    uint64_t frame_status{0};

    /**
     * Thermal shutdown counter. Please refer to the firmware documentation for
     * more information.
     */
    uint8_t shutdown_countdown{0};

    /**
     * Shot limiting counter. Please refer to the firmware documentation for
     * more information.
     */
    uint8_t shot_limiting_countdown{0};

    /**
     * The current frame ID.
     *
     * @warning Members variables: use with caution, some of these will become
     * private.
     */
    int64_t frame_id{-1};

    /**
     * The associated sensor info for this lidar frame
     */
    std::shared_ptr<SensorInfo> sensor_info;

    /** The default constructor creates an invalid 0 x 0 frame. */
    OUSTER_API_FUNCTION
    LidarFrame();

    /**
     * Initialize a frame with fields configured from the data format.
     *
     * @param[in] format data format
     */
    OUSTER_DEPRECATED_MSG("LidarFrame(std::shared_ptr<SensorInfo>)",
                          OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    LidarFrame(const DataFormat& format);

    /**
     * Initialize a frame with fields configured for the LEGACY udp profile.
     *
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] w horizontal resolution, i.e. the number of measurements per
     *              frame.
     *
     * Note, the number of columns per packet is set to the default
     * (DEFAULT_COLUMNS_PER_PACKET).
     */
    OUSTER_DEPRECATED_MSG("LidarFrame(h, w, field_types, columns_per_packet)",
                          OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    LidarFrame(size_t h, size_t w);

    /**
     * Initialize a frame with fields configured as default for the provided
     * SensorInfo's configuration
     *
     * @param[in] sensor_info description of sensor to create frame for
     */
    OUSTER_DEPRECATED_MSG("LidarFrame(std::shared_ptr<SensorInfo>)",
                          OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    LidarFrame(const SensorInfo& sensor_info);

    /**
     * Initialize a frame configured as default for the provided
     * SensorInfo's configuration
     *
     * @param[in] sensor_info a shared_ptr to the SensorInfo object
     */
    OUSTER_API_FUNCTION
    LidarFrame(std::shared_ptr<SensorInfo> sensor_info);

    /**
     * Initialize a frame for the given sensor info with either the default for
     * the provided configuration or provided fields.
     *
     * @param[in] sensor_info a shared_ptr to the SensorInfo object
     * @param[in] field_types fields to use in the lidar frame
     */
    OUSTER_API_FUNCTION
    LidarFrame(std::shared_ptr<SensorInfo> sensor_info, const std::vector<FieldType>& field_types);

    /**
     * Initialize a frame with the default fields for a particular udp profile.
     *
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] w horizontal resolution, i.e. the number of measurements per
     *              frame.
     * @param[in] profile udp profile.
     * @param[in] columns_per_packet The number of columns per packet,
     *                               this argument is optional.
     */
    OUSTER_DEPRECATED_MSG("LidarFrame(h, w, field_types, columns_per_packet)",
                          OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    LidarFrame(size_t h, size_t w, UDPProfileLidar profile, size_t columns_per_packet);

    /**
     * Initialize a frame with a custom set of fields.
     *
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] w horizontal resoulution, i.e. the number of measurements per
     *              frame.
     * @param[in] field_types types and names of fields to add to the frame.
     * @param[in] columns_per_packet The number of columns per packet,
     *                               this argument is optional.
     */
    OUSTER_API_FUNCTION
    LidarFrame(size_t h, size_t w, const LidarFrameFieldTypes& field_types,
               size_t columns_per_packet);

    /**
     * Initialize a lidar frame from another lidar frame.
     *
     * @param[in] other The other lidar frame to initialize from.
     */
    OUSTER_API_FUNCTION
    LidarFrame(const LidarFrame& other);

    /**
     * Initialize a lidar frame from another with only the indicated fields.
     * Casts, zero pads or removes fields from the original frame if necessary.
     *
     * @throw std::invalid_argument if field dimensions are incompatible
     *
     * @param[in] other The other lidar frame to initialize from.
     * @param[in] fields Fields to have in new lidar frame.
     */
    OUSTER_API_FUNCTION
    LidarFrame(const LidarFrame& other, const LidarFrameFieldTypes& fields);

    /**
     * Initialize a lidar frame by moving from another lidar frame.
     * @param[in] other The other lidar frame to initialize from.
     * @note after this operation the moved-from object is unusable.
     */
    OUSTER_API_FUNCTION
    LidarFrame(LidarFrame&& other) noexcept;

    /**
     * Copy assignment.
     *
     * @param[in] other The lidar frame to copy from.
     */
    OUSTER_API_FUNCTION
    LidarFrame& operator=(const LidarFrame& other);

    /**
     * Move assignment
     *
     * @param[in] other The lidar frame to move from.
     * @note after this operation the moved-from object is unusable.
     */
    OUSTER_API_FUNCTION
    LidarFrame& operator=(LidarFrame&& other) noexcept;

    /**
     * Lidar frame destructor.
     */
    OUSTER_API_FUNCTION
    ~LidarFrame();

    /**
     * Get frame shot limiting status
     *
     * @return true if sensor is shot limiting
     */
    OUSTER_API_FUNCTION
    ShotLimitingStatus shot_limiting() const;

    /**
     * Get frame thermal shutdown status
     *
     * @return true if sensor is in thermal shutdown state.
     */
    OUSTER_API_FUNCTION
    ThermalShutdownStatus thermal_shutdown() const;

    /**
     * @defgroup CoreLidarFrameField Access fields in a lidar frame
     * Access a lidar data field.
     *
     * @throws std::invalid_argument if T does not match the runtime field type.
     * @throws std::out_of_range if field is not present
     *
     * @tparam T The type parameter T must match the dynamic type of the field.
     * See the constructor documentation for expected field types or query
     * dynamically for generic operations.
     *
     * @param[in] f the field to view.
     *
     * @return a view of the field data.
     */

    /**
     * @copydoc CoreLidarFrameField
     *
     * @par Supported \c T
     * \c field<T>() supports element types \c uint8_t, \c int8_t, \c uint16_t,
     * \c int16_t, \c uint32_t, \c int32_t, \c uint64_t, \c int64_t, \c float,
     * \c float16_t, and \c double.
     */
    // NOTE: defined inline on purpose. Apple Clang 13 has a mangling bug where
    // explicit instantiation of these member function templates produces an
    // ABI-incompatible mangle for the dependent default StrideType expression
    // in Eigen::Ref. Defining inline forces every TU to implicitly instantiate
    // from the same header, which keeps mangling consistent across the
    // library and its consumers
    template <typename T>
    OUSTER_API_FUNCTION Eigen::Ref<img_t<T>> field(const std::string& f) {
        return field(f);
    }

    /**
     * @copydoc CoreLidarFrameField
     *
     * @par Supported \c T
     * Same element types as the non-const \c field() template above.
     */
    template <typename T>
    OUSTER_API_FUNCTION Eigen::Ref<const img_t<T>> field(const std::string& f) const {
        return field(f);
    }

    /**
     * @defgroup CoreLidarFrameFieldString Access fields in a lidar frame
     * Access a lidar data field.
     *
     * @throws std::out_of_range if field is not present
     *
     * @param[in] name string key of the field to access
     * @return Field reference of the requested field
     */

    /**
     * @copydoc CoreLidarFrameFieldString
     */
    OUSTER_API_FUNCTION
    Field& field(const std::string& name);

    /**
     * @copydoc CoreLidarFrameFieldString
     */
    OUSTER_API_FUNCTION
    const Field& field(const std::string& name) const;

    /**
     * Check if a field exists
     *
     * @param[in] name string key of the field to check
     *
     * @return true if the lidar frame has the field, else false
     */
    OUSTER_API_FUNCTION
    bool has_field(const std::string& name) const;

    /**
     * Add a new zero-filled field to lidar frame.
     *
     * @throw std::invalid_argument if key duplicates a preexisting field, or
     *        if flags dimensional requirements are not met
     *
     * @param[in] name string key of the field to add
     * @param[in] d descriptor of the field to add
     * @param[in] field_class class to be assigned to the field, e.g.
     * PIXEL_FIELD
     *
     * @return field
     */
    OUSTER_API_FUNCTION
    Field& add_field(const std::string& name, FieldDescriptor d,
                     FieldClass field_class = FieldClass::PIXEL_FIELD);

    /**
     * Add a new zero-filled field to lidar frame.
     *
     * @throw std::invalid_argument if key duplicates a preexisting field
     *
     * @param[in] type Descriptor of the field to add.
     *
     * @return field The value of the field added.
     */
    OUSTER_API_FUNCTION
    Field& add_field(const FieldType& type);

    /**
     * Release the field and remove it from lidar frame
     *
     * @throw std::invalid_argument if field under key does not exist
     *
     * @param[in] name string key of the field to remove
     *
     * @return field The deleted field.
     */
    OUSTER_API_FUNCTION
    Field del_field(const std::string& name);

    /**
     * Get the type of the specified field.
     *
     * @param[in] name the string key of the field to query.
     *
     * @return the type associated with the field.
     */
    OUSTER_API_FUNCTION
    FieldType field_type(const std::string& name) const;

    /**
     * Get the FieldType of all fields in the frame
     *
     * @return The type associated with every field in the frame.
     */
    OUSTER_API_FUNCTION
    LidarFrameFieldTypes field_types() const;

    /**
     * Reference to the internal fields map
     *
     * @return The unordered map of field type and field.
     */
    OUSTER_API_FUNCTION
    std::unordered_map<std::string, Field>& fields();

    /** @copydoc fields() */
    OUSTER_API_FUNCTION
    const std::unordered_map<std::string, Field>& fields() const;

    /**
     * Reference to the internal objects map
     *
     * @return The unordered map of names and object lists.
     */
    OUSTER_API_FUNCTION
    std::unordered_map<std::string, std::vector<Object>>& objects();

    /** @copydoc objects() */
    OUSTER_API_FUNCTION
    const std::unordered_map<std::string, std::vector<Object>>& objects() const;

    /**
     * Access the measurement timestamp headers.
     *
     * @return a view of timestamp as a w-element vector.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint64_t>> timestamp();

    /**
     * @copydoc timestamp()
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint64_t>> timestamp() const;

    /**
     * Access the packet timestamp headers (usually host time).
     *
     * @return a view of timestamp as a vector with w / columns-per-packet
     * elements.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint64_t>> packet_timestamp();

    /**
     * Access the host timestamp headers (usually host time).
     *
     * @return a view of timestamp as a vector with w / columns-per-packet
     * elements.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint64_t>> packet_timestamp() const;

    /**
     * Access the packet alert flags headers.
     *
     * @return a view of timestamp as a vector with w / columns-per-packet
     * elements.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint8_t>> alert_flags();

    /**
     * Access the packet alert flags headers.
     *
     * @return a view of timestamp as a vector with w / columns-per-packet
     * elements.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint8_t>> alert_flags() const;

    /**
     * Access the zone monitor states.
     *
     * If zone states are not present, returns an empty view.
     *
     * @return a view of zone monitor states
     */
    OUSTER_API_FUNCTION
    ArrayView1<ZoneState> zones();

    /**
     * Access the zone monitor states.
     *
     * If zone states are not present, returns an empty view.
     *
     * @return a view of zone monitor states
     */
    OUSTER_API_FUNCTION
    ConstArrayView1<ZoneState> zones() const;

    /**
     * Return the first valid packet timestamp from any packet type.
     *
     * @return the first valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_first_valid_packet_timestamp() const;

    /**
     * Return the first valid packet timestamp for a specific packet type.
     *
     * @param[in] stream the packet type to query
     * @return the first valid packet timestamp for the given type
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_first_valid_packet_timestamp(PacketType stream) const;

    /**
     * Return the last valid packet timestamp from any packet type.
     *
     * @return the last valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_last_valid_packet_timestamp() const;

    /**
     * Return the last valid packet timestamp for a specific packet type.
     *
     * @param[in] stream the packet type to query
     * @return the last valid packet timestamp for the given type
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_last_valid_packet_timestamp(PacketType stream) const;

    /**
     * Return the min valid packet timestamp from any packet type.
     *
     * @return the min valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_min_valid_packet_timestamp() const;

    /**
     * Return the min valid packet timestamp for a specific packet type.
     *
     * @param[in] stream the packet type to query
     * @return the min valid packet timestamp for the given type
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_min_valid_packet_timestamp(PacketType stream) const;

    /**
     * Return the max valid packet timestamp from any packet type.
     *
     * @return the max valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_max_valid_packet_timestamp() const;

    /**
     * Return the max valid packet timestamp for a specific packet type.
     *
     * @param[in] stream the packet type to query
     * @return the max valid packet timestamp for the given type
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_max_valid_packet_timestamp(PacketType stream) const;

    /**
     * Return the first valid lidar packet timestamp
     *
     * @return the first valid lidar packet timestamp, 0 if none available
     */
    OUSTER_DEPRECATED_MSG("LidarFrame::get_first_valid_packet_timestamp(PacketType::Lidar)",
                          OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    uint64_t get_first_valid_lidar_packet_timestamp() const;

    /**
     * Return the last valid lidar packet timestamp
     *
     * @return the last valid lidar packet timestamp, 0 if none available
     */
    OUSTER_DEPRECATED_MSG("LidarFrame::get_last_valid_packet_timestamp(PacketType::Lidar)",
                          OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    uint64_t get_last_valid_lidar_packet_timestamp() const;

    /**
     * Return the first valid column index
     *
     * @return the first valid column index
     * @throw std::runtime_error if no valid columns are available
     */
    OUSTER_API_FUNCTION
    int get_first_valid_column() const;

    /**
     * Return the last valid column index
     *
     * @return the last valid column index
     * @throw std::runtime_error if no valid columns are available
     */
    OUSTER_API_FUNCTION
    int get_last_valid_column() const;

    /**
     * Access the measurement id headers.
     *
     * @return a view of measurement ids as a w-element vector.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint16_t>> measurement_id();

    /** @copydoc measurement_id() */
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint16_t>> measurement_id() const;

    /**
     * Access the measurement status headers.
     *
     * @return a view of measurement statuses as a w-element vector.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint32_t>> status();

    /** @copydoc status() */
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint32_t>> status() const;

    /**
     * Access the array of body-to-world transforms (per each timestamp). Cast
     * to ArrayView3<double> in order to access as 3d
     * @return 3d field of homogenous pose matrices, shaped (w, 4, 4).
     */
    OUSTER_API_FUNCTION
    Field& body_to_world();

    /** @copydoc body_to_world() */
    OUSTER_API_FUNCTION
    const Field& body_to_world() const;

    /**
     * Access the array of body-to-world transforms (per each timestamp). Cast
     * to ArrayView3<double> in order to access as 3d
     *
     * @deprecated
     *
     * @return 3d field of homogenous pose matrices, shaped (w, 4, 4).
     */
    OUSTER_DEPRECATED_MSG("LidarFrame::body_to_world()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    Field& pose();

    /** @copydoc pose() */
    OUSTER_DEPRECATED_MSG("LidarFrame::body_to_world()", OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    const Field& pose() const;

    /**
     * @brief Set accessor for a column pose
     *
     * @param[in] index The column index to set the pose for.
     * @param[in] pose The pose to set for the column as a 4x4 matrix
     * @throw std::out_of_range if index is out of bounds
     */
    OUSTER_API_FUNCTION
    void set_column_pose(int index, const Matrix4dR& pose);

    /**
     * @brief Get accessor for a column pose
     *
     * @param[in] index The column index to get the pose for.
     * @throw std::out_of_range if index is out of bounds
     *
     * @return The pose of the column (index) as a 4x4 matrix.
     */
    OUSTER_API_FUNCTION
    Matrix4dR get_column_pose(int index) const;

    /**
     * Assess completeness of frame.
     * @param[in] window The column window to use for validity assessment
     * @return whether all columns within given column window were valid
     */
    OUSTER_API_FUNCTION
    bool complete(ColumnWindow window) const;

    /**
     * Assess completeness of frame.
     * @return whether all columns within the column window were valid
     * @throw std::runtime_error if the LidarFrame does not have an associated
     * sensor_info
     */
    OUSTER_API_FUNCTION
    bool complete() const;

    /**
     * Returns the number of lidar packets needed to fill this frame.
     *
     * @return the number of packets
     */
    OUSTER_API_FUNCTION
    size_t packet_count() const;

    /**
     * Equality for frames.
     *
     * @param[in] other The other frame to compare to.
     *
     * @return if a == b.
     */
    OUSTER_API_FUNCTION
    bool equals(const LidarFrame& other) const;

   private:
    nonstd::optional<uint64_t> get_first_valid_lidar_packet_timestamp_internal() const;
    nonstd::optional<uint64_t> get_last_valid_lidar_packet_timestamp_internal() const;
    nonstd::optional<uint64_t> get_first_valid_imu_packet_timestamp_internal() const;
    nonstd::optional<uint64_t> get_last_valid_imu_packet_timestamp_internal() const;
    nonstd::optional<uint64_t> get_min_valid_lidar_packet_timestamp_internal() const;
    nonstd::optional<uint64_t> get_max_valid_lidar_packet_timestamp_internal() const;
    nonstd::optional<uint64_t> get_min_valid_imu_packet_timestamp_internal() const;
    nonstd::optional<uint64_t> get_max_valid_imu_packet_timestamp_internal() const;
    nonstd::optional<uint64_t> get_valid_zone_packet_timestamp_internal() const;
};

/**
 * Equality for LidarFrame.
 *
 * @param[in] a The first frame to compare.
 * @param[in] b The second frame to compare.
 *
 * @return if a == b.
 */
OUSTER_API_FUNCTION
bool operator==(const LidarFrame& a, const LidarFrame& b);

/**
 * Inequality for LidarFrame.
 *
 * @param[in] a The first frame to compare.
 * @param[in] b The second frame to compare.
 *
 * @return if a != b.
 */
OUSTER_API_FUNCTION
bool operator!=(const LidarFrame& a, const LidarFrame& b);

/**
 * Get string representation of lidar frame field types.
 *
 * @param[in] field_types The field types to get the string representation of.
 *
 * @return string representation of the lidar frame field types.
 */
OUSTER_API_FUNCTION
std::string to_string(const LidarFrameFieldTypes& field_types);

/**
 * Get the lidar frame field types from data format
 *
 * @param[in] format data format
 * @param[in] fw_version firmware version of sensor
 *
 * @return The lidar frame field types
 */
OUSTER_API_FUNCTION
LidarFrameFieldTypes get_field_types(const DataFormat& format, const Version& fw_version);

/**
 * Get the lidar frame field types from sensor info
 *
 * @param[in] info The sensor info to get the lidar frame field types from.
 *
 * @return The lidar frame field types
 */
OUSTER_API_FUNCTION
LidarFrameFieldTypes get_field_types(const SensorInfo& info);

/**
 * Get string representation of a lidar frame.
 *
 * @param[in] lidar_frame The lidar frame to get the string representation of.
 *
 * @return string representation of the lidar frame.
 */
OUSTER_API_FUNCTION
std::string to_string(const LidarFrame& lidar_frame);

/** \defgroup ouster_core_lidar_frame_operators Ouster Client lidar_scan.h
 * Operators
 * @{
 */

/** @}*/

/** \defgroup ouster_core_destagger Ouster Client lidar_scan.h
 * @{
 */
/**
 * Generate a destaggered version of a channel field.
 *
 * In the default staggered representation, each column corresponds to a single
 * timestamp. In the destaggered representation, each column corresponds to a
 * single azimuth angle, compensating for the azimuth offset of each beam.
 *
 * Destaggering is used for visualizing lidar data as an image or for algorithms
 * that exploit the structure of the lidar data, such as beam_uniformity in
 * ouster_viz, or computer vision algorithms.
 *
 * @tparam T the datatype of the channel field.
 *
 * @param[in] img the channel field.
 * @param[in] pixel_shift_by_row offsets, usually queried from the sensor.
 * @param[in] inverse when true, undo a previous destagger by reapplying the
 *            per-row pixel shifts (i.e. return to the original staggered
 * layout).
 *
 * @return destaggered version of the image.
 */
template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& pixel_shift_by_row, bool inverse = false);

/**
 * Generate a staggered version of a channel field.
 *
 * @tparam T the datatype of the channel field.
 *
 * @param[in] img the channel field.
 * @param[in] pixel_shift_by_row offsets, usually queried from the sensor.
 *
 * @return staggered version of the image.
 */
template <typename T>
inline img_t<T> stagger(const Eigen::Ref<const img_t<T>>& img,
                        const std::vector<int>& pixel_shift_by_row) {
    return destagger(img, pixel_shift_by_row, true);
}

/**
 * Return the column timestamp for a pixel in destaggered image coordinates.
 *
 * Column timestamps are indexed by staggered column. This function maps
 * destaggered image coordinates back to the corresponding staggered column
 * before indexing @p column_timestamps.
 *
 * @param[in] row destaggered row index.
 * @param[in] col destaggered column index.
 * @param[in] pixel_shift_by_row per-row destagger offsets from sensor metadata.
 * @param[in] column_timestamps staggered column timestamps with one entry per
 * column.
 *
 * @return column timestamp in nanoseconds for the mapped staggered column.
 *
 * @throws std::invalid_argument if row/col are out of range.
 */
OUSTER_API_FUNCTION
uint64_t column_timestamp_at_destaggered_pixel(
    size_t row, size_t col, const std::vector<int>& pixel_shift_by_row,
    const Eigen::Ref<const Eigen::Array<uint64_t, Eigen::Dynamic, 1>>& column_timestamps);
/** @}*/

/**
 * Parse lidar packets into a LidarFrame.
 *
 * Make a function that batches a single frame (revolution) of data to a
 * LidarFrame.
 */
class OUSTER_API_CLASS FrameBatcher {
    /**
     * Comparator for ordering packets in the cache by frame id.
     */
    struct OUSTER_API_CLASS PacketComparator {
        PacketFormat pf;
        /**
         * Initialize the packet comparator with a packet format.
         *
         * @param[in] pf_ the packet format to use for comparing packets.
         */
        OUSTER_API_FUNCTION
        PacketComparator(const PacketFormat& pf_) : pf(pf_) {}
        /**
         * Compare two packets by their frame id.
         *
         * @param[in] a the first packet to compare.
         * @param[in] b the second packet to compare.
         *
         * @return true if a's frame id is less than b's frame id, false
         * otherwise.
         */
        OUSTER_API_FUNCTION
        bool operator()(const std::unique_ptr<Packet>& a, const std::unique_ptr<Packet>& b) const {
            return pf.frame_id_difference(pf.frame_id(a->buf.data()), pf.frame_id(b->buf.data())) <
                   0;
        }
    };

   public:
    PacketFormat pf;  ///< The packet format object used for decoding

   private:
    size_t max_cache_size_{4};  ///< The maximum number of packets to hold in
                                ///< the cache (minimum is 1)
    uint16_t next_valid_m_id_{0};
    uint16_t next_headers_m_id_{0};
    // A priority queue that prioritizes packets by frame id
    std::priority_queue<std::unique_ptr<Packet>, std::deque<std::unique_ptr<Packet>>,
                        PacketComparator>
        cache_;
    int64_t finished_frame_id_{-1};
    int64_t last_frame_id_{-1};  // the frame id of the most recently released frame
    int64_t last_init_id_;
    std::shared_ptr<SensorInfo> sensor_info_;
    bool reset_frame_{true};  ///< Whether the next packet should start a new frame

    size_t expected_lidar_packets_;
    size_t expected_imu_packets_{0};
    size_t expected_zone_packets_{0};
    size_t batched_lidar_packets_{0};
    size_t batched_imu_packets_{0};
    size_t batched_zone_packets_{0};

    void parse_by_col(const uint8_t* packet_buf, LidarFrame& lidar_frame);
    void parse_by_block(const uint8_t* packet_buf, LidarFrame& lidar_frame);

    void cache_packet(const Packet& packet);
    void batch_packet(const Packet& packet, LidarFrame& lidar_frame);
    void start_frame(int64_t f_id, const uint8_t* packet_buf, LidarFrame& lidar_frame);
    void finalize_frame(LidarFrame& lidar_frame);

    bool handle_init_id_change(const Packet& packet, LidarFrame& lidar_frame);
    bool batch_with_caching(const Packet& packet, LidarFrame& lidar_frame);
    void batch_lidar_packet(const LidarPacket& packet, LidarFrame& lidar_frame);
    void batch_imu_packet(const ImuPacket& packet, LidarFrame& lidar_frame);
    void batch_zone_packet(const ZonePacket& packet, LidarFrame& lidar_frame);
    bool check_frame_complete(const LidarFrame& lidar_frame) const;

    size_t dropped_packets_{0};

   public:
    /**
     * Create a batcher given information about the frame and packet format.
     *
     * @param[in] info sensor metadata returned from the client.
     */
    OUSTER_API_FUNCTION
    explicit FrameBatcher(const SensorInfo& info);

    /**
     * Create a batcher given information about the frame and packet format.
     *
     * @param[in] info sensor metadata returned from the client.
     */
    OUSTER_API_FUNCTION
    explicit FrameBatcher(const std::shared_ptr<SensorInfo>& info);

    FrameBatcher(const FrameBatcher&) = delete;
    FrameBatcher& operator=(const FrameBatcher&) = delete;

    /**
     * Move constructor.
     */
    OUSTER_API_FUNCTION
    FrameBatcher(FrameBatcher&&) = default;

    /**
     * Move assignment operator.
     */
    OUSTER_API_FUNCTION
    FrameBatcher& operator=(FrameBatcher&&) = delete;

    /**
     * Add a packet to the frame.
     * This will ignore legacy IMU packets.
     *
     * @param[in] packet an ouster data packet.
     * @param[in] lidar_frame lidar frame to populate.
     *
     * @return true when the provided lidar frame is ready to use.
     */
    OUSTER_DEPRECATED_MSG(FrameBatcher batch(Packet, LidarFrame),
                          OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
    OUSTER_API_FUNCTION
    bool operator()(const Packet& packet, LidarFrame& lidar_frame);

    /**
     * Add a packet to the frame.
     * This will ignore legacy IMU packets.
     *
     * @param[in] packet an ouster data packet.
     * @param[in] lidar_frame lidar frame to populate.
     * @throw std::invalid_argument if the provided frame is a different
     * resolution or if the number of columns per packet in the provided frame
     * is different than the sensor info the batcher was initialized with.
     * @throw std::runtime_error if the batcher would produce a frame id that is
     * not strictly greater than the previous frame id for the same sensor init
     * id when using a FUSA packet header type.
     *
     * @return true when the provided lidar frame is ready to use.
     */
    OUSTER_API_FUNCTION
    bool batch(const Packet& packet, LidarFrame& lidar_frame);

    /**
     * Reset the batcher and clear any cached packets.
     */
    OUSTER_API_FUNCTION
    void reset();

    /**
     * Get the number of packets batched into the frame so far. Returns 0 if a
     * frame was just released from operator().
     *
     * @return the number of packets batched into the frame so far
     */
    OUSTER_API_FUNCTION
    size_t batched_packets() const;

    /**
     * Get the number of packets dropped by the batcher. A dropped packet is a
     * packet that was not included in a frame released from operator() due to
     * arriving after a new frame started or arriving too late to be included in
     * a frame before it was released.
     *
     * @return the number of packets dropped by the batcher
     */
    OUSTER_API_FUNCTION
    size_t dropped_packets() const;

    /**
     * Set the maximum number of packets to hold in the cache before a new
     * packet triggers the batcher finalizes the current frame.
     *
     * @param[in] max_cache_size the maximum number of packets to hold in the
     * cache (minimum is 1)
     */
    OUSTER_API_FUNCTION
    void set_max_cache_size(size_t max_cache_size);

    /**
     * Get the maximum number of packets to hold in the cache before a new
     * packet triggers the batcher finalizes the current frame.
     *
     * @return the maximum number of packets.
     */
    OUSTER_API_FUNCTION
    size_t get_max_cache_size() const;
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster

#include "ouster/core/impl/lidar_frame_impl.h"

namespace ouster {
namespace sdk {
namespace core {

OUSTER_DEPRECATED_TYPE(LidarScan, LidarFrame, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(LidarScanFieldTypes, LidarFrameFieldTypes,
                       OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_DEPRECATED_TYPE(ScanBatcher, FrameBatcher, OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)

}  // namespace core
}  // namespace sdk
}  // namespace ouster
