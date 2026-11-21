/**
 * @file
 * @brief Common frame base for lidar and camera data
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <memory>
#include <nonstd/optional.hpp>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "ouster/core/deprecation.h"
#include "ouster/core/field.h"
#include "ouster/core/field_type.h"
#include "ouster/core/object.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

// Forward declaration; the full definition lives in sensor_info.h. Only a
// shared_ptr to it is stored here, so the declaration is sufficient.
struct SensorInfo;

class Frame;

namespace impl {

/**
 * Associate @p info with @p frame without knowing the frame's concrete type.
 *
 * This is the only write path that is not checked at compile time, and it
 * exists solely for generic code that re-points a frame of unknown kind at a
 * different SensorInfo instance (e.g. MultiFrameSetSource's sensor dedup).
 * Prefer TypedFrame::set_sensor_info(), which rejects the wrong metadata type
 * at compile time.
 *
 * @param[in,out] frame The frame to associate the metadata with.
 * @param[in] info The metadata to associate, or nullptr to clear it.
 *
 * @throws std::invalid_argument if @p info does not describe @p frame's sensor
 *         kind (see Frame::accepts_sensor_info()).
 */
OUSTER_API_FUNCTION
void assign_sensor_info(Frame& frame, std::shared_ptr<SensorInfo> info);

}  // namespace impl

/**
 * Polymorphic base for sensor frames (LidarFrame, CameraFrame, ...).
 *
 * Holds named fields, per-timestamp poses, and packet/timestamp headers shared
 * by lidar scans and camera images. Concrete frame types add sensor-specific
 * metadata and accessors.
 */
class OUSTER_API_CLASS Frame {
   protected:
    /**
     * The sensor metadata (calibration) associated with this frame.
     *
     * Held polymorphically as the shared SensorInfo base so that generic code
     * operating on a Frame can read the shared calibration (extrinsics, serial
     * number, ...) without knowing the concrete sensor kind. Protected rather
     * than public because assigning it cannot be checked at compile time; see
     * sensor_info() and TypedFrame::set_sensor_info().
     */
    std::shared_ptr<SensorInfo> sensor_info_;

    std::unordered_map<std::string, Field> fields_;
    std::unordered_map<std::string, std::vector<Object>> objects_;

    /**
     * Build the descriptor for a field of @p field_type sized to this frame.
     *
     * @param[in] field_type The field type to describe.
     *
     * @return the descriptor, with leading dimensions derived from the field
     *         class (h x w for pixel fields, w for column fields, the packet
     *         count for packet fields, none for frame fields).
     */
    OUSTER_API_FUNCTION
    FieldDescriptor get_field_type_descriptor(const FieldType& field_type) const;

    /**
     * The number of packets used to produce a full frame given the width in
     * pixels and the number of columns per packet.
     */
    size_t packet_count_{0};

    /**
     * Per-timestamp measurement times. Length is the number of columns for a
     * LidarFrame and 1 for a CameraFrame, not Frame::w in general.
     */
    Field timestamp_;

    /**
     * Per-timestamp body-to-world transforms, shaped (N, 4, 4) where N matches
     * timestamp_. Length is the number of columns for a LidarFrame and 1 for a
     * CameraFrame, not Frame::w in general.
     */
    Field body_to_world_;

    /**
     * Per-packet host timestamps (usually host time). For a LidarFrame the
     * length is the number of packets (w / columns-per-packet). A CameraFrame
     * has no packets; its length is allowed to vary and is used purely to carry
     * per-frame timing information (e.g. for visualization).
     */
    Field receive_timestamp_;

    /**
     * Per-timestamp validity/status word. Length matches timestamp_ (the number
     * of columns for a LidarFrame, 1 for a CameraFrame). Bit 0 indicates the
     * timestamp is valid. Used by the shared packet-timestamp accessors to map
     * timestamps to packets: columns-per-packet is status().size() /
     * receive_timestamp().size(), so the same logic works for lidar and camera.
     */
    Field status_;

    /**
     * Earliest packet timestamp whose corresponding status entries include a
     * valid column, or nullopt if none are valid. Derived types may combine
     * this with additional streams (e.g. IMU/zone for LidarFrame).
     *
     * @return the first valid packet timestamp, if any.
     */
    OUSTER_API_FUNCTION
    nonstd::optional<uint64_t> first_valid_receive_timestamp_internal() const;

    /** @copydoc first_valid_receive_timestamp_internal() but for the last. */
    OUSTER_API_FUNCTION
    nonstd::optional<uint64_t> last_valid_receive_timestamp_internal() const;

    /** @copydoc first_valid_receive_timestamp_internal() but for the min. */
    OUSTER_API_FUNCTION
    nonstd::optional<uint64_t> min_valid_receive_timestamp_internal() const;

    /** @copydoc first_valid_receive_timestamp_internal() but for the max. */
    OUSTER_API_FUNCTION
    nonstd::optional<uint64_t> max_valid_receive_timestamp_internal() const;

    /**
     * Allocate timestamp_ and body_to_world_ for count poses and initialize
     * each pose to identity.
     *
     * @param[in] count Number of timestamps / poses to allocate.
     * @param[in] timestamp_class FieldClass for timestamp_.
     * @param[in] pose_class FieldClass for body_to_world_.
     */
    OUSTER_API_FUNCTION
    void init_timestamps_and_poses(size_t count, FieldClass timestamp_class, FieldClass pose_class);

   public:
    template <typename T>
    using Header = Eigen::Array<T, Eigen::Dynamic, 1>;  ///< Header typedef

    /**
     * Virtual destructor. Frame is a polymorphic base; instances are commonly
     * held and destroyed through a std::shared_ptr<Frame>.
     */
    OUSTER_API_FUNCTION
    virtual ~Frame();

    /**
     * Default constructor.
     */
    OUSTER_API_FUNCTION
    Frame() = default;

    /**
     * Copy constructor.
     */
    OUSTER_API_FUNCTION
    Frame(const Frame&) = default;

    /**
     * Copy assignment operator.
     */
    OUSTER_API_FUNCTION
    Frame& operator=(const Frame&) = default;

    /**
     * Move constructor.
     */
    OUSTER_API_FUNCTION
    Frame(Frame&&) = default;

    /**
     * Move assignment operator.
     */
    OUSTER_API_FUNCTION
    Frame& operator=(Frame&&) = default;

    /**
     * Deep-copy this frame, preserving its concrete derived type.
     *
     * @return a shared pointer to a newly allocated copy of the frame.
     */
    OUSTER_API_FUNCTION
    virtual std::shared_ptr<Frame> clone() const = 0;

    /**
     * Human-readable string representation of this frame.
     *
     * Virtual so that a base Frame& (or shared_ptr<Frame>) prints the concrete
     * frame type's details (LidarFrame, CameraFrame, ...). The base
     * implementation prints the shared header (dimensions) followed by a
     * per-field dump. Derived types override to prepend sensor-specific
     * information.
     *
     * @return the string representation of the frame.
     */
    OUSTER_API_FUNCTION
    virtual std::string to_string() const;

    /**
     * Value equality against another frame.
     *
     * Virtual so that comparing two frames held as a base Frame& (or
     * shared_ptr<Frame>, as FrameSet does) accounts for the concrete type's
     * state instead of silently comparing only the members declared here. The
     * base implementation requires both operands to have the same concrete
     * type, then compares the dimensions, packet count, timestamp / pose /
     * status headers, fields, and objects. Derived types override to add their
     * own members and must call this base implementation first.
     *
     * The associated sensor_info() is deliberately not compared. It is held by
     * shared_ptr, so comparing it would either test pointer identity — making
     * two frames carrying identical data unequal merely because they were read
     * from separate files — or require a deep metadata comparison that callers
     * of frame equality rarely want. Compare sensor_info() explicitly when the
     * calibration matters.
     *
     * @param[in] other The frame to compare against.
     *
     * @return true if this frame and @p other are of the same concrete type and
     *         hold equal data.
     */
    OUSTER_API_FUNCTION
    virtual bool equals(const Frame& other) const;

    /**
     * The sensor metadata (calibration) associated with this frame.
     *
     * Returns the shared SensorInfo base, so generic code operating on a Frame
     * can read the calibration common to every sensor kind (extrinsics, serial
     * number, w()/h(), the cached XYZLut) without downcasting. Concrete frame
     * types hide this with a same-named accessor returning their own metadata
     * type — see TypedFrame::sensor_info().
     *
     * @return the associated SensorInfo, or nullptr if none is set.
     */
    OUSTER_API_FUNCTION
    const std::shared_ptr<SensorInfo>& sensor_info() const;

    /**
     * Whether @p info describes this frame's sensor kind.
     *
     * The base accepts any metadata; TypedFrame<InfoT> narrows it to InfoT.
     * Used by impl::assign_sensor_info() to validate the one write path that
     * cannot be checked at compile time.
     *
     * @param[in] info The metadata to test.
     *
     * @return true if @p info may be associated with this frame.
     */
    OUSTER_API_FUNCTION
    virtual bool accepts_sensor_info(const SensorInfo& info) const;

    /**
     * Grants impl::assign_sensor_info() write access to sensor_info_.
     *
     * There is deliberately no setter here: a base Frame cannot verify the
     * sensor kind at compile time, so metadata is assigned either through the
     * typed TypedFrame::set_sensor_info() or, for genuinely generic code,
     * through the runtime-checked impl::assign_sensor_info().
     *
     * @param[in,out] frame The frame to assign metadata to.
     * @param[in] info The metadata to assign.
     */
    friend void impl::assign_sensor_info(Frame& frame, std::shared_ptr<SensorInfo> info);

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
     * @param[in] name the name of the field to view.
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
    OUSTER_API_FUNCTION Eigen::Ref<img_t<T>> field(const std::string& name) {
        return field(name);
    }

    /**
     * @copydoc CoreLidarFrameField
     *
     * @par Supported \c T
     * Same element types as the non-const \c field() template above.
     */
    template <typename T>
    OUSTER_API_FUNCTION Eigen::Ref<const img_t<T>> field(const std::string& name) const {
        return field(name);
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
     * @param[in] desc descriptor of the field to add
     * @param[in] field_class class to be assigned to the field, e.g.
     * PIXEL_FIELD
     * @param[in] zero If true, zero the field otherwise memory is uninitialized.
     *
     * @return field
     */
    OUSTER_API_FUNCTION
    Field& add_field(const std::string& name, FieldDescriptor desc,
                     FieldClass field_class = FieldClass::PIXEL_FIELD, bool zero = true);

    /**
     * Add a new zero-filled field to lidar frame.
     *
     * @throw std::invalid_argument if key duplicates a preexisting field
     *
     * @param[in] type Descriptor of the field to add.
     * @param[in] zero If true, zero the field otherwise memory is uninitialized.
     *
     * @return field The value of the field added.
     */
    OUSTER_API_FUNCTION
    Field& add_field(const FieldType& type, bool zero = true);

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
     * Returns the number of lidar packets needed to fill this frame.
     *
     * @return the number of packets
     */
    OUSTER_API_FUNCTION
    size_t packet_count() const;

    /**
     * Access the measurement timestamp header.
     *
     * @return a view of timestamps. Length is w for a LidarFrame and 1 for a
     *         CameraFrame.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint64_t>> timestamp();

    /** @copydoc timestamp() */
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint64_t>> timestamp() const;

    /**
     * Access the per-packet host timestamp headers (usually host time).
     *
     * @return a view of packet timestamps. Length is w / columns-per-packet for
     *         a LidarFrame; for a CameraFrame the length is allowed to vary.
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint64_t>> receive_timestamp();

    /** @copydoc receive_timestamp() */
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint64_t>> receive_timestamp() const;

    /**
     * @deprecated Renamed to receive_timestamp(); this alias forwards to it and
     * will be removed in a future release.
     *
     * @return a view of packet timestamps (see receive_timestamp()).
     */
    OUSTER_DEPRECATED_MSG(receive_timestamp(), OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint64_t>> packet_timestamp() {
        return receive_timestamp();
    }

    /** @copydoc packet_timestamp() */
    OUSTER_DEPRECATED_MSG(receive_timestamp(), OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint64_t>> packet_timestamp() const {
        return receive_timestamp();
    }

    /**
     * Access the per-timestamp status header (bit 0 = valid).
     *
     * @return a view of statuses. Length matches timestamp() (w for a
     *         LidarFrame, 1 for a CameraFrame).
     */
    OUSTER_API_FUNCTION
    Eigen::Ref<Header<uint32_t>> status();

    /** @copydoc status() */
    OUSTER_API_FUNCTION
    Eigen::Ref<const Header<uint32_t>> status() const;

    /**
     * Return the first valid packet timestamp.
     *
     * The base implementation maps timestamps to packets using status() and
     * receive_timestamp(). Derived types may override to combine additional
     * streams (e.g. LidarFrame folds in IMU and zone packets).
     *
     * @return the first valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    virtual uint64_t get_first_valid_receive_timestamp() const;

    /**
     * Return the last valid packet timestamp.
     *
     * @return the last valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    virtual uint64_t get_last_valid_receive_timestamp() const;

    /**
     * Return the min valid packet timestamp.
     *
     * @return the min valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    virtual uint64_t get_min_valid_receive_timestamp() const;

    /**
     * Return the max valid packet timestamp.
     *
     * @return the max valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    virtual uint64_t get_max_valid_receive_timestamp() const;

    /**
     * @deprecated Renamed to get_first_valid_receive_timestamp(); this alias
     * forwards to it and will be removed in a future release.
     *
     * @return the first valid packet timestamp
     */
    OUSTER_DEPRECATED_MSG(get_first_valid_receive_timestamp(), OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    uint64_t get_first_valid_packet_timestamp() const {
        return get_first_valid_receive_timestamp();
    }

    /**
     * @deprecated Renamed to get_last_valid_receive_timestamp(); this alias
     * forwards to it and will be removed in a future release.
     *
     * @return the last valid packet timestamp
     */
    OUSTER_DEPRECATED_MSG(get_last_valid_receive_timestamp(), OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    uint64_t get_last_valid_packet_timestamp() const {
        return get_last_valid_receive_timestamp();
    }

    /**
     * @deprecated Renamed to get_min_valid_receive_timestamp(); this alias
     * forwards to it and will be removed in a future release.
     *
     * @return the min valid packet timestamp
     */
    OUSTER_DEPRECATED_MSG(get_min_valid_receive_timestamp(), OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    uint64_t get_min_valid_packet_timestamp() const {
        return get_min_valid_receive_timestamp();
    }

    /**
     * @deprecated Renamed to get_max_valid_receive_timestamp(); this alias
     * forwards to it and will be removed in a future release.
     *
     * @return the max valid packet timestamp
     */
    OUSTER_DEPRECATED_MSG(get_max_valid_receive_timestamp(), OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    uint64_t get_max_valid_packet_timestamp() const {
        return get_max_valid_receive_timestamp();
    }

    /**
     * Return the first valid column index.
     *
     * Scans the status() header (bit 0 = valid) for the first valid entry.
     * Works for any Frame: a LidarFrame has one status per column, while a
     * CameraFrame has a single status entry (index 0).
     *
     * @return the first valid column index
     * @throw std::runtime_error if no valid columns are available
     */
    OUSTER_API_FUNCTION
    int get_first_valid_column() const;

    /**
     * Return the last valid column index.
     *
     * Scans the status() header (bit 0 = valid) for the last valid entry.
     * Works for any Frame: a LidarFrame has one status per column, while a
     * CameraFrame has a single status entry (index 0).
     *
     * @return the last valid column index
     * @throw std::runtime_error if no valid columns are available
     */
    OUSTER_API_FUNCTION
    int get_last_valid_column() const;

    /**
     * Access the array of body-to-world transforms (per each timestamp). Cast
     * to ArrayView3<double> in order to access as 3d.
     *
     * @return 3d field of homogeneous pose matrices, shaped (N, 4, 4) where N
     *         is the number of timestamps (w for lidar, 1 for camera).
     */
    OUSTER_API_FUNCTION
    Field& body_to_world();

    /** @copydoc body_to_world() */
    OUSTER_API_FUNCTION
    const Field& body_to_world() const;

    /**
     * Set the pose at the given timestamp index.
     *
     * Index is a column index for LidarFrame and 0 for CameraFrame. Bounds are
     * the pose array's first dimension, not Frame::w.
     *
     * @param[in] index The timestamp index to set the pose for.
     * @param[in] pose The pose to set as a 4x4 matrix.
     * @throw std::out_of_range if index is out of bounds
     */
    OUSTER_API_FUNCTION
    void set_column_pose(int index, const Matrix4dR& pose);  // TODO[tws] rename?

    /**
     * Get the pose at the given timestamp index.
     *
     * Index is a column index for LidarFrame and 0 for CameraFrame. Bounds are
     * the pose array's first dimension, not Frame::w.
     *
     * @param[in] index The timestamp index to get the pose for.
     * @throw std::out_of_range if index is out of bounds
     *
     * @return The pose as a 4x4 matrix.
     */
    OUSTER_API_FUNCTION
    Matrix4dR get_column_pose(int index) const;  // TODO[tws] rename?

    /**
     * Get the FieldType of all fields in the frame
     *
     * @return The type associated with every field in the frame.
     */
    OUSTER_API_FUNCTION
    LidarFrameFieldTypes field_types() const;
};

/**
 * Mixin giving a concrete frame type a statically typed view of its metadata.
 *
 * Covariant return types do not apply to std::shared_ptr, so TypedFrame
 * emulates them by *hiding* Frame::sensor_info() with a same-named accessor
 * returning std::shared_ptr<InfoT>. The accessor therefore has one name across
 * the whole hierarchy, and its result type follows the static type of the
 * expression: a Frame yields SensorInfo, a LidarFrame yields LidarInfo, and a
 * CameraFrame yields CameraInfo.
 *
 * The matching set_sensor_info() accepts only InfoT, so associating metadata of
 * the wrong sensor kind is a compile error rather than a silently null
 * accessor. There is no setter on Frame itself precisely because a base Frame
 * could not make that check (see impl::assign_sensor_info()).
 *
 * The mixin stores nothing and adds no virtuals beyond the
 * accepts_sensor_info() override, so the metadata still lives on the Frame base
 * and generic code that holds a Frame is unaffected.
 *
 * @tparam InfoT the concrete SensorInfo subtype describing this frame's sensor
 *               (LidarInfo for LidarFrame, CameraInfo for CameraFrame).
 */
template <typename InfoT>
class TypedFrame : public Frame {
    // std::is_base_of_v is C++17; library code is C++14.
    // NOLINTNEXTLINE(modernize-type-traits)
    static_assert(std::is_base_of<SensorInfo, InfoT>::value,
                  "TypedFrame's InfoT must derive from SensorInfo");

   public:
    /** The concrete metadata type describing this frame's sensor. */
    using info_type = InfoT;

    /**
     * The sensor metadata associated with this frame, as its concrete type.
     *
     * Hides Frame::sensor_info(). Use `frame.Frame::sensor_info()` to reach the
     * base accessor from a concrete frame type.
     *
     * @return a shared_ptr to the associated InfoT, or nullptr if no sensor
     *         info is set (or, defensively, if it holds a different subtype).
     */
    std::shared_ptr<InfoT> sensor_info() const {
        return std::dynamic_pointer_cast<InfoT>(sensor_info_);
    }

    /**
     * Associate sensor metadata with this frame.
     *
     * Accepts only InfoT, so metadata of the wrong sensor kind is rejected at
     * compile time.
     *
     * @param[in] info the metadata to associate, or nullptr to clear it.
     */
    void set_sensor_info(std::shared_ptr<InfoT> info) {
        sensor_info_ = std::move(info);
    }

    /** @copydoc Frame::accepts_sensor_info() */
    bool accepts_sensor_info(const SensorInfo& info) const override {
        return dynamic_cast<const InfoT*>(&info) != nullptr;
    }
};

/**
 * Get a string representation of a frame.
 *
 * Dispatches polymorphically to Frame::to_string(), so it works on a base
 * Frame& regardless of the concrete frame type (LidarFrame, CameraFrame, ...).
 *
 * @param[in] frame The frame to get the string representation of.
 *
 * @return string representation of the frame.
 */
OUSTER_API_FUNCTION
std::string to_string(const Frame& frame);

/**
 * Equality for frames.
 *
 * Dispatches polymorphically to Frame::equals(), so it works on base Frame
 * references regardless of the concrete frame type: two LidarFrames compare
 * their lidar headers, two CameraFrames compare the shared frame state, and a
 * lidar frame is never equal to a camera frame.
 *
 * @param[in] a The first frame to compare.
 * @param[in] b The second frame to compare.
 *
 * @return true if a == b.
 */
OUSTER_API_FUNCTION
bool operator==(const Frame& a, const Frame& b);

/**
 * Inequality for frames.
 *
 * @param[in] a The first frame to compare.
 * @param[in] b The second frame to compare.
 *
 * @return true if a != b.
 */
OUSTER_API_FUNCTION
bool operator!=(const Frame& a, const Frame& b);

namespace impl {

/**
 * Append a human-readable, per-field dump of @p fields to @p os (one line per
 * field with its name, dtype, shape, and either a numeric min/mean/max summary
 * or a text preview for CHAR fields).
 *
 * Shared by the Frame / LidarFrame / CameraFrame to_string() implementations so
 * that field dumps render identically across frame types.
 *
 * @param[in] fields The fields map to render.
 * @param[in,out] out_stream The output stream to append to.
 */
OUSTER_API_FUNCTION
void append_frame_fields(const std::unordered_map<std::string, Field>& fields,
                         std::ostream& out_stream);

}  // namespace impl

}  // namespace core
}  // namespace sdk
}  // namespace ouster
