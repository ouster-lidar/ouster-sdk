/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <memory>
#include <vector>

#include "os_sensor/lidar_scan_stream_generated.h"
#include "ouster/lidar_scan.h"

namespace ouster {
namespace osf {

// Encoded single PNG buffer
using ScanChannelData = std::vector<uint8_t>;

// Encoded PNG buffers
using ScanData = std::vector<ScanChannelData>;

// FieldTypes container
using LidarScanFieldTypes =
    std::vector<std::pair<sensor::ChanField, sensor::ChanFieldType>>;

/**
 * libpng only versions for Encode/Decode LidarScan to PNG buffers
 */

// ========== Decode Functions ===================================

/**
 * Decode the PNG buffers into LidarScan object. This is a dispatch function to
 * the specific decoding functions.
 *
 * @param lidar_scan the output object that will be filled as a result of
 *                   decoding
 * @param scan_data  PNG buffers to decode
 * @param px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecode(LidarScan& lidar_scan, const ScanData& scan_data,
                const std::vector<int>& px_offset);

/// Decoding eUDP LidarScan
// TODO[pb]: Make decoding of just some fields from scan data?? Not now ...
bool scanDecodeFieldsSingleThread(LidarScan& lidar_scan,
                                  const ScanData& scan_data,
                                  const std::vector<int>& px_offset);

/// Decoding eUDP LidarScan, multithreaded version
bool scanDecodeFields(LidarScan& lidar_scan, const ScanData& scan_data,
                      const std::vector<int>& px_offset);

/**
 * Decode a single field to lidar_scan
 *
 * @param lidar_scan the output object that will be filled as a result of
 *                   decoding
 * @param scan_data  PNG buffers to decode
 * @param scan_idx   index in `scan_data` of the beginning of field buffers
 * @param field_type  the field of `lidar_scan` to fill in with the docoded
 *                    result
 * @param px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @return false (0) if operation is successful true (1) if error occured
 */
bool fieldDecode(
    LidarScan& lidar_scan, const ScanData& scan_data, size_t scan_idx,
    const std::pair<sensor::ChanField, sensor::ChanFieldType> field_type,
    const std::vector<int>& px_offset);

/**
 * Decode multiple fields to lidar_scan
 *
 * @param lidar_scan the output object that will be filled as a result of
 *                   decoding
 * @param scan_data  PNG buffers to decode, sequentially in the order of
 *                   field_types
 * @param scan_idxs  a vector of indices in `scan_data` of the beginning of
 *                   field buffers that will be decoded. `field_types.size()`
 *                   should be equal to `scan_idxs.size()` i.e. we need to
 *                   provide the index for every field type in
 *                   field_types where it's encoded data located
 * @param field_types a vector of filed_types of lidar scan to decode
 * @param px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @return false (0) if operation is successful true (1) if error occured
 */
bool fieldDecodeMulti(LidarScan& lidar_scan, const ScanData& scan_data,
                      const std::vector<size_t>& scan_idxs,
                      const LidarScanFieldTypes& field_types,
                      const std::vector<int>& px_offset);

template <typename T>
bool decode8bitImage(Eigen::Ref<img_t<T>> img,
                     const ScanChannelData& channel_buf);

template <typename T>
bool decode8bitImage(Eigen::Ref<img_t<T>> img,
                     const ScanChannelData& channel_buf,
                     const std::vector<int>& px_offset);

/**
 * Decode single PNG buffer (channel_buf) of 16 bit Gray encoding into
 * img
 *
 * @param img         the output img that will be filled as a result of
 *                    decoding
 * @param channel_buf single PNG buffer to decode
 * @param px_offset   pixel shift per row used to reconstruct staggered range
 *                    image form
 * @return false (0) if operation is successful true (1) if error occured
 */
template <typename T>
bool decode16bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset);

template <typename T>
bool decode16bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf);

/**
 * Decode single PNG buffer (channel_buf) of 24 bit RGB (8 bit) encoding into
 * img object.
 *
 * @param img         the output img that will be filled as a result of
 *                    decoding
 * @param channel_buf single PNG buffer to decode
 * @param px_offset   pixel shift per row used to reconstruct staggered range
 *                    image form
 * @return false (0) if operation is successful true (1) if error occured
 */
template <typename T>
bool decode24bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset);

template <typename T>
bool decode24bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf);

/**
 * Decode single PNG buffer (channel_buf) of 32 bit RGBA (8 bit) encoding into
 * img object.
 *
 * @param img         the output img that will be filled as a result of
 *                    decoding
 * @param channel_buf single PNG buffer to decode
 * @param px_offset   pixel shift per row used to reconstruct staggered range
 *                    image form
 * @return false (0) if operation is successful true (1) if error occured
 */
template <typename T>
bool decode32bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset);

template <typename T>
bool decode32bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf);

/**
 * Decode single PNG buffer (channel_buf) of 64 bit RGBA (16 bit) encoding into
 * img object.
 *
 * @param img         the output img that will be filled as a result of
 *                    decoding
 * @param channel_buf single PNG buffer to decode
 * @param px_offset   pixel shift per row used to reconstruct staggered range
 *                    image form
 * @return false (0) if operation is successful true (1) if error occured
 */
template <typename T>
bool decode64bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf,
                      const std::vector<int>& px_offset);

template <typename T>
bool decode64bitImage(Eigen::Ref<img_t<T>> img,
                      const ScanChannelData& channel_buf);

// ========== Encode Functions ===================================

/**
 * Encode LidarScan to PNG buffers storing all field_types present in an object.
 *
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destaggered LidarScan data
 * @return encoded PNG buffers, empty() if error occured.
 */
ScanData scanEncode(const LidarScan& lidar_scan,
                    const std::vector<int>& px_offset);

/**
 * Encode the lidar scan fields to PNGs channel buffers (ScanData).
 * Single-threaded implementation.
 *
 * @param lidar_scan a lidar scan object to encode
 * @param px_offset  pixel shift per row used to construct de-staggered range
 *                   image form
 * @return encoded PNGs in ScanData in order of field_types
 */
ScanData scanEncodeFieldsSingleThread(const LidarScan& lidar_scan,
                                      const std::vector<int>& px_offset,
                                      const LidarScanFieldTypes& field_types);

/**
 * Encode the lidar scan fields to PNGs channel buffers (ScanData).
 * Multi-threaded implementation.
 *
 * @param lidar_scan a lidar scan object to encode
 * @param px_offset  pixel shift per row used to construct de-staggered range
 *                   image form
 * @return encoded PNGs in ScanData in order of field_types
 */
ScanData scanEncodeFields(const LidarScan& lidar_scan,
                          const std::vector<int>& px_offset,
                          const LidarScanFieldTypes& field_types);

/**
 * Encode a single lidar scan field to PNGs channel buffer and place it to a
 * specified `scan_data[scan_idx]` place
 *
 * @param lidar_scan a lidar scan object to encode
 * @param field_type a filed_type of lidar scan to encode
 * @param px_offset  pixel shift per row used to construct de-staggered range
 *                   image form
 * @param scan_data channel buffers storage for the encoded lidar_scan
 * @param scan_idx index in `scan_data` of the beginning of field buffers where
 *                 the result of encoding will be inserted
 * @return false (0) if operation is successful true (1) if error occured
 */
bool fieldEncode(
    const LidarScan& lidar_scan,
    const std::pair<sensor::ChanField, sensor::ChanFieldType> field_type,
    const std::vector<int>& px_offset, ScanData& scan_data, size_t scan_idx);

/**
 * Encode multiple lidar scan fields to PNGs channel buffers and insert them to
 * a specified places `scan_idxs` in `scan_data`.
 *
 * @param lidar_scan a lidar scan object to encode
 * @param field_types a vector of filed_types of lidar scan to encode
 * @param px_offset  pixel shift per row used to construct de-staggered range
 *                   image form
 * @param scan_data channel buffers storage for the encoded lidar_scan
 * @param scan_idxs a vector of indices in `scan_data` of the beginning of field
 *                 buffers where the result of encoding will be inserted.
 *                 `field_types.size()` should be equal to `scan_idxs.size()`
 * @return false (0) if operation is successful true (1) if error occured
 */
bool fieldEncodeMulti(const LidarScan& lidar_scan,
                      const LidarScanFieldTypes& field_types,
                      const std::vector<int>& px_offset, ScanData& scan_data,
                      const std::vector<size_t>& scan_idxs);

template <typename T>
bool encode8bitImage(ScanChannelData& res_buf,
                     const Eigen::Ref<const img_t<T>>& img,
                     const std::vector<int>& px_offset);

template <typename T>
bool encode8bitImage(ScanChannelData& res_buf,
                     const Eigen::Ref<const img_t<T>>& img);

/**
 * Encode img object into a 16 bit, Gray, PNG buffer.
 *
 * @param res_buf    the output buffer with a single encoded PNG
 * @param img        the image object to encode
 * @param px_offset  pixel shift per row used to destagger img data
 * @return false (0) if operation is successful, true (1) if error occured
 */
template <typename T>
bool encode16bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset);

/**
 * Encode 2D image of a typical lidar scan field channel into a 16 bit, Gray,
 * PNG buffer.
 *
 * @param res_buf    the output buffer with a single encoded PNG
 * @param img        2D image or a single LidarScan field data
 * @return false (0) if operation is successful, true (1) if error occured
 */
template <typename T>
bool encode16bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img);

/**
 * Encode 2D image of a typical lidar scan field channel into a 32 bit, RGBA,
 * PNG buffer.
 *
 * @param res_buf    the output buffer with a single encoded PNG
 * @param img        2D image or a single LidarScan field data
 * @param px_offset  pixel shift per row used to destagger img data
 * @return false (0) if operation is successful, true (1) if error occured
 */
template <typename T>
bool encode32bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset);

template <typename T>
bool encode32bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img);

/**
 * Encode 2D image of a typical lidar scan field channel into a 24 bit, RGB,
 * PNG buffer.
 *
 * @param res_buf    the output buffer with a single encoded PNG
 * @param img        2D image or a single LidarScan field data
 * @param px_offset  pixel shift per row used to destagger img data
 * @return false (0) if operation is successful, true (1) if error occured
 */
template <typename T>
bool encode24bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset);

template <typename T>
bool encode24bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img);

/**
 * Encode 2D image of a typical lidar scan field channel into a 64 bit, RGBA,
 * PNG buffer.
 *
 * @param res_buf    the output buffer with a single encoded PNG
 * @param img        2D image or a single LidarScan field data
 * @param px_offset  pixel shift per row used to destagger img data
 * @return false (0) if operation is successful, true (1) if error occured
 */
template <typename T>
bool encode64bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img,
                      const std::vector<int>& px_offset);

template <typename T>
bool encode64bitImage(ScanChannelData& res_buf,
                      const Eigen::Ref<const img_t<T>>& img);

// =================== Save to File Functions ====================

/**
 * Save PNG encoded scan channel buffer to the PNG file.
 *
 * @param channel_buf single PNG buffer to decode
 * @param filename    file name of output PNG image
 * @return false (0) if operation is successful, true (1) if error occured
 */
bool saveScanChannel(const ScanChannelData& channel_buf,
                     const std::string& filename);

}  // namespace osf
}  // namespace ouster
