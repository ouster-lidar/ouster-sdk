#pragma once

#include <memory>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/osf/common.h"
#include "ouster/osf/version.h"
#include "ouster/types.h"

namespace ouster {
namespace OSF {

/**
 * NOTE: For available lidar frame modes and it's composition please see the
 * OSF_FRAME_MODE definition with comments and test results provided in
 * ouster_osf/osfCommon.fbs file.
 */

// Encoded single PNG buffer
using ScanChannelData = std::vector<uint8_t>;

// Encoded PNG buffers
using ScanData = std::vector<ScanChannelData>;

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
 * @param frame_mode specifies the encoding format of scan_data buffers
 * @param range_multiplier quantization factor used in OSF_32 mode only
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecode(LidarScan& lidar_scan, const ScanData& scan_data,
                const std::vector<int>& px_offset,
                const OSF_FRAME_MODE frame_mode,
                const int range_multiplier = OSF::RANGE_MULTIPLIER_DEFAULT);

/**
 * Decode the PNG buffers of OSF_32 encoding into LidarScan object.
 *
 * @param lidar_scan the output object that will be filled as a result of
 *                   decoding
 * @param scan_data  PNG buffers to decode, for OSF_32 scan_data.size() == 1
 * @param px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @param range_multiplier quantization factor
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecodeOSF32(
    LidarScan& lidar_scan, const ScanData& scan_data,
    const std::vector<int>& px_offset,
    const int range_multiplier = OSF::RANGE_MULTIPLIER_DEFAULT);

/**
 * Decode the PNG buffers of OSF_56 encoding into LidarScan object.
 *
 * @param lidar_scan the output object that will be filled as a result of
 *                   decoding
 * @param scan_data  PNG buffers to decode, for OSF_56 scan_data.size() == 3
 * @param px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecodeOSF56(LidarScan& lidar_scan, const ScanData& scan_data,
                     const std::vector<int>& px_offset);

/**
 * Decode the PNG buffers of OSF_40RI encoding into LidarScan object.
 *
 * @param lidar_scan the output object that will be filled as a result of
 *                   decoding
 * @param scan_data  PNG buffers to decode, for OSF_40RI scan_data.size() == 2
 * @param px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecodeOSF40RI(LidarScan& lidar_scan, const ScanData& scan_data,
                       const std::vector<int>& px_offset);

/**
 * Decode the PNG buffers of OSF_40RN encoding into LidarScan object.
 *
 * @param lidar_scan the output object that will be filled as a result of
 *                   decoding
 * @param scan_data  PNG buffers to decode, for OSF_40RN scan_data.size() == 2
 * @param px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecodeOSF40RN(LidarScan& lidar_scan, const ScanData& scan_data,
                       const std::vector<int>& px_offset);

/**
 * Decode the PNG buffers of OSF_72 encoding into LidarScan object.
 *
 * @param lidar_scan the output object that will be filled as a result of
 *                   decoding
 * @param scan_data  PNG buffers to decode, for OSF_72 scan_data.size() == 4
 * @param px_offset  pixel shift per row used to reconstruct staggered range
 *                   image form
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecodeOSF72(LidarScan& lidar_scan, const ScanData& scan_data,
                     const std::vector<int>& px_offset);

/**
 * Decode single PNG buffer (channel_buf) of OSF_32 encoding into LidarScan
 * object. OSF_32 buffer encoded as RGBA, 8 bit.
 *
 * @param lidar_scan  the output object that will be filled as a result of
 *                    decoding
 * @param channel_buf single PNG buffer to decode for OSF_32
 * @param px_offset   pixel shift per row used to reconstruct staggered range
 *                    image form
 * @param range_multiplier quantization factor
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecodeOSF32Channel(
    LidarScan& lidar_scan, const ScanChannelData& channel_buf,
    const std::vector<int>& px_offset,
    const int range_multiplier = OSF::RANGE_MULTIPLIER_DEFAULT);

/**
 * Decode single PNG buffer (channel_buf) of 16 bit Gray encoding into
 * LidarScan object to the specified channel_index key. Used in OSF_56, OSF_72,
 * OSF_40RI and OSF_40RN frame modes.
 *
 * @param lidar_scan  the output object that will be filled as a result of
 *                    decoding
 * @param channel_buf single PNG buffer to decode
 * @param px_offset   pixel shift per row used to reconstruct staggered range
 *                    image form
 * @param channel_index key of output LidarScan where to store decoded data
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecode16bitChannel(LidarScan& lidar_scan,
                            const ScanChannelData& channel_buf,
                            const std::vector<int>& px_offset,
                            const LidarScan::LidarScanIndex channel_index =
                                LidarScan::LidarScanIndex::INTENSITY);

/**
 * Decode single PNG buffer (channel_buf) of 24 bit RGB (8 bit) encoding into
 * LidarScan object to the specified channel_index key. Usually used for Range
 * channel in OSF_40+ modes.
 *
 * @param lidar_scan  the output object that will be filled as a result of
 *                    decoding
 * @param channel_buf single PNG buffer to decode
 * @param px_offset   pixel shift per row used to reconstruct staggered range
 *                    image form
 * @param channel_index key of output LidarScan where to store decoded data
 * @return false (0) if operation is successful true (1) if error occured
 */
bool scanDecode24bitChannel(LidarScan& lidar_scan,
                            const ScanChannelData& channel_buf,
                            const std::vector<int>& px_offset,
                            const LidarScan::LidarScanIndex channel_index =
                                LidarScan::LidarScanIndex::RANGE);

// ========== Encode Functions ===================================

/**
 * Encode LidarScan to PNG buffers with the specified lidar_frame encoding mode.
 * This is a dispatch function to the specific encoding functions.
 *
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destaggered LidarScan data
 * @param frame_mode specifies the encoding format of PNG buffers to generate
 * @param range_multiplier quantization factor used for OSF_32 mode only and can
 * be left as default for other encoding modes.
 * @return encoded PNG buffers, empty() if error occured.
 */
ScanData scanEncode(
    const LidarScan& lidar_scan, const std::vector<int>& px_offset,
    const OSF_FRAME_MODE frame_mode = OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_32,
    const int range_multiplier = OSF::RANGE_MULTIPLIER_DEFAULT);

/**
 * Encode LidarScan to OSF_32 mode PNG buffer.
 *
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destagger LidarScan data
 * @param range_multiplier quantization factor for OSF_32 Range channel
 * @return encoded PNG buffers, empty() if error occured.
 */
ScanData scanEncodeOSF32(
    const LidarScan& lidar_scan, const std::vector<int>& px_offset,
    const int range_multiplier = OSF::RANGE_MULTIPLIER_DEFAULT);

/**
 * Encode LidarScan to OSF_40RI mode PNG buffers.
 *
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destagger LidarScan data
 * @return encoded PNG buffers, empty() if error occured.
 */
ScanData scanEncodeOSF40RI(const LidarScan& lidar_scan,
                           const std::vector<int>& px_offset);

/**
 * Encode LidarScan to OSF_40RN mode PNG buffers.
 *
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destagger LidarScan data
 * @return encoded PNG buffers, empty() if error occured.
 */
ScanData scanEncodeOSF40RN(const LidarScan& lidar_scan,
                           const std::vector<int>& px_offset);

/**
 * Encode LidarScan to OSF_56 mode PNG buffers.
 *
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destagger LidarScan data
 * @return encoded PNG buffers, empty() if error occured.
 */
ScanData scanEncodeOSF56(const LidarScan& lidar_scan,
                         const std::vector<int>& px_offset);

/**
 * Encode LidarScan to OSF_72 mode PNG buffers.
 *
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destagger LidarScan data
 * @return encoded PNG buffers, empty() if error occured.
 */
ScanData scanEncodeOSF72(const LidarScan& lidar_scan,
                         const std::vector<int>& px_offset);

/**
 * Encode LidarScan to a OSF_32 PNG buffer.
 *
 * @param res_buf    the output buffer with a single encoded PNG in OSF_32 mode
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destagger LidarScan data
 * @param range_multiplier quantization factor for OSF_32 Range channel
 * @return encoded PNG buffer, empty() if error occured.
 */
bool scanEncodeOSF32Channel(
    ScanChannelData& res_buf, const LidarScan& lidar_scan,
    const std::vector<int>& px_offset,
    const int range_multiplier = OSF::RANGE_MULTIPLIER_DEFAULT);

/**
 * Encode LidarScan channel_index key into a 16 bit, Gray, PNG buffer.
 *
 * @param res_buf    the output buffer with a single encoded PNG
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destagger LidarScan data
 * @param channel_index key of LidarScan object to encode
 * @return false (0) if operation is successful, true (1) if error occured
 */
bool scanEncode16bitChannel(ScanChannelData& res_buf,
                            const LidarScan& lidar_scan,
                            const std::vector<int>& px_offset,
                            const LidarScan::LidarScanIndex channel_index =
                                LidarScan::LidarScanIndex::INTENSITY);

/**
 * Encode LidarScan channel_index key into a 24 bit, RGB (8 bit) PNG buffer.
 *
 * @param res_buf    the output buffer with a single encoded PNG
 * @param lidar_scan the LidarScan object to encode
 * @param px_offset  pixel shift per row used to destagger LidarScan data
 * @param channel_index key of LidarScan object to encode
 * @return false (0) if operation is successful, true (1) if error occured
 */
bool scanEncode24bitChannel(ScanChannelData& res_buf,
                            const LidarScan& lidar_scan,
                            const std::vector<int>& px_offset,
                            const LidarScan::LidarScanIndex channel_index =
                                LidarScan::LidarScanIndex::RANGE);

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

}  // namespace OSF
}  // namespace ouster