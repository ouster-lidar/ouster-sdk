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
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "ouster/core/defaults.h"
#include "ouster/core/field.h"
#include "ouster/core/field_type.h"
#include "ouster/core/frame.h"
#include "ouster/core/lidar_info.h"
#include "ouster/core/object.h"
#include "ouster/core/packet.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"
#include "ouster/core/zone_state.h"

namespace ouster {
namespace sdk {
namespace core {
// Forward declaration
class LidarInfo;

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
class OUSTER_API_CLASS LidarFrame : public TypedFrame<LidarInfo> {
   public:
    template <typename T>
    using Header = Frame::Header<T>;  ///< Header typedef

   private:
    // Required special case "fields"
    Field measurement_id_;

    /**
     * The alert flags field, from eUDP packet headers.
     */
    Field alert_flags_;

   public:
    /**
     * The current frame ID.
     *
     * @warning Member variables: use with caution, some of these will become
     * private.
     */
    int64_t frame_id{-1};

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

    // The LidarInfo associated with this frame is reached through the
    // inherited TypedFrame<LidarInfo>::sensor_info() / set_sensor_info().

    /** The default constructor creates an invalid 0 x 0 frame. */
    OUSTER_API_FUNCTION
    LidarFrame();

    /**
     * Initialize a frame configured as default for the provided
     * LidarInfo's configuration
     *
     * @param[in] sensor_info a shared_ptr to the LidarInfo object
     */
    OUSTER_API_FUNCTION
    explicit LidarFrame(std::shared_ptr<LidarInfo> sensor_info);

    /**
     * Initialize a frame for the given sensor info with either the default for
     * the provided configuration or provided fields.
     *
     * @param[in] sensor_info a shared_ptr to the LidarInfo object
     * @param[in] field_types fields to use in the lidar frame
     */
    OUSTER_API_FUNCTION
    LidarFrame(std::shared_ptr<LidarInfo> sensor_info, const std::vector<FieldType>& field_types);

    /**
     * Initialize a frame with a custom set of fields.
     *
     * @param[in] h vertical resolution, i.e. the number of channels.
     * @param[in] w horizontal resolution, i.e. the number of measurements per
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
    ~LidarFrame() override;

    /**
     * Deep-copy this lidar frame as a base Frame pointer.
     *
     * @return a shared pointer to a newly allocated LidarFrame copy.
     */
    OUSTER_API_FUNCTION
    std::shared_ptr<Frame> clone() const override;

    /**
     * Get a string representation of this lidar frame.
     *
     * @return the string representation of the lidar frame.
     */
    OUSTER_API_FUNCTION
    std::string to_string() const override;

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
    uint64_t get_first_valid_receive_timestamp() const override;

    /**
     * Return the first valid packet timestamp for a specific packet type.
     *
     * @param[in] stream the packet type to query
     * @return the first valid packet timestamp for the given type
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_first_valid_receive_timestamp(PacketType stream) const;

    /**
     * Return the last valid packet timestamp from any packet type.
     *
     * @return the last valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_last_valid_receive_timestamp() const override;

    /**
     * Return the last valid packet timestamp for a specific packet type.
     *
     * @param[in] stream the packet type to query
     * @return the last valid packet timestamp for the given type
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_last_valid_receive_timestamp(PacketType stream) const;

    /**
     * Return the min valid packet timestamp from any packet type.
     *
     * @return the min valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_min_valid_receive_timestamp() const override;

    /**
     * Return the min valid packet timestamp for a specific packet type.
     *
     * @param[in] stream the packet type to query
     * @return the min valid packet timestamp for the given type
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_min_valid_receive_timestamp(PacketType stream) const;

    /**
     * Return the max valid packet timestamp from any packet type.
     *
     * @return the max valid packet timestamp
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_max_valid_receive_timestamp() const override;

    /**
     * Return the max valid packet timestamp for a specific packet type.
     *
     * @param[in] stream the packet type to query
     * @return the max valid packet timestamp for the given type
     * @throw std::runtime_error if no valid packets are available
     */
    OUSTER_API_FUNCTION
    uint64_t get_max_valid_receive_timestamp(PacketType stream) const;

    // Un-hide the deprecated no-argument packet_timestamp aliases inherited from
    // Frame; the PacketType overloads declared below would otherwise hide them.
    using Frame::get_first_valid_packet_timestamp;
    using Frame::get_last_valid_packet_timestamp;
    using Frame::get_max_valid_packet_timestamp;
    using Frame::get_min_valid_packet_timestamp;

    /**
     * @deprecated Renamed to get_first_valid_receive_timestamp(PacketType);
     * this alias forwards to it and will be removed in a future release.
     *
     * @param[in] stream the packet type to query
     * @return the first valid packet timestamp for the given type
     */
    OUSTER_DEPRECATED_MSG(get_first_valid_receive_timestamp(stream),
                          OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    uint64_t get_first_valid_packet_timestamp(PacketType stream) const {
        return get_first_valid_receive_timestamp(stream);
    }

    /**
     * @deprecated Renamed to get_last_valid_receive_timestamp(PacketType);
     * this alias forwards to it and will be removed in a future release.
     *
     * @param[in] stream the packet type to query
     * @return the last valid packet timestamp for the given type
     */
    OUSTER_DEPRECATED_MSG(get_last_valid_receive_timestamp(stream),
                          OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    uint64_t get_last_valid_packet_timestamp(PacketType stream) const {
        return get_last_valid_receive_timestamp(stream);
    }

    /**
     * @deprecated Renamed to get_min_valid_receive_timestamp(PacketType);
     * this alias forwards to it and will be removed in a future release.
     *
     * @param[in] stream the packet type to query
     * @return the min valid packet timestamp for the given type
     */
    OUSTER_DEPRECATED_MSG(get_min_valid_receive_timestamp(stream),
                          OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    uint64_t get_min_valid_packet_timestamp(PacketType stream) const {
        return get_min_valid_receive_timestamp(stream);
    }

    /**
     * @deprecated Renamed to get_max_valid_receive_timestamp(PacketType);
     * this alias forwards to it and will be removed in a future release.
     *
     * @param[in] stream the packet type to query
     * @return the max valid packet timestamp for the given type
     */
    OUSTER_DEPRECATED_MSG(get_max_valid_receive_timestamp(stream),
                          OUSTER_DEPRECATED_LAST_SUPPORTED_2_0)
    OUSTER_API_FUNCTION
    uint64_t get_max_valid_packet_timestamp(PacketType stream) const {
        return get_max_valid_receive_timestamp(stream);
    }

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
     * @copydoc Frame::equals()
     *
     * Extends the base comparison with the lidar-specific state: the frame id
     * and status, the measurement id and alert flag headers, and the thermal
     * shutdown / shot limiting countdowns.
     */
    OUSTER_API_FUNCTION
    bool equals(const Frame& other) const override;

   private:
    nonstd::optional<uint64_t> get_first_valid_imu_receive_timestamp_internal() const;
    nonstd::optional<uint64_t> get_last_valid_imu_receive_timestamp_internal() const;
    nonstd::optional<uint64_t> get_min_valid_imu_receive_timestamp_internal() const;
    nonstd::optional<uint64_t> get_max_valid_imu_receive_timestamp_internal() const;
    nonstd::optional<uint64_t> get_valid_zone_receive_timestamp_internal() const;
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
LidarFrameFieldTypes get_field_types(const LidarInfo& info);

/**
 * Get string representation of a lidar frame.
 *
 * @param[in] lidar_frame The lidar frame to get the string representation of.
 *
 * @return string representation of the lidar frame.
 */
OUSTER_API_FUNCTION
std::string to_string(const LidarFrame& lidar_frame);

/**
 * Convert a LidarFrame to the packets that would produce it.
 *
 * @param[in] ls The lidar frame to produce packets from.
 * @param[in] packet_format The packet format to produce packets in.
 * @param[in] init_id Initialization id to write into packets.
 * @param[in] prod_sn Product serial number to write into packets.
 *
 * @throws std::invalid_argument if packet_format is null.
 * @throws std::invalid_argument if the frame width and columns_per_packet are
 *         inconsistent with receive_timestamp() length.
 * @return the list of packets in the frame
 */
OUSTER_API_FUNCTION
std::vector<Packet> frame_to_packets(
    const LidarFrame& ls, const std::shared_ptr<ouster::sdk::core::PacketFormat> packet_format,
    uint32_t init_id, uint64_t prod_sn);

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
    std::shared_ptr<LidarInfo> sensor_info_;
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
    explicit FrameBatcher(const LidarInfo& info);

    /**
     * Create a batcher given information about the frame and packet format.
     *
     * @param[in] info sensor metadata returned from the client.
     */
    OUSTER_API_FUNCTION
    explicit FrameBatcher(const std::shared_ptr<LidarInfo>& info);

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
     * Finalize a lidar frame that was not completely filled.
     * Generally used at the end of a pcap file.
     * @param[in] lidar_frame lidar frame to finalize
     */
    OUSTER_API_FUNCTION
    void finalize_frame(LidarFrame& lidar_frame);

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
