/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include "ouster/lidar_scan.h"
#include "ouster/osf/lidarscan_encoder.h"
#include "ouster/visibility.h"

namespace ouster {
namespace osf {

/**
 * Effect of png_set_compression(comp level):
 * - (no png out):     2s, n/a
 * - comp level 1:    39s, 648M  (60% speedup vs default, 10% size increase)
 * - comp level 2:    38s, 643M
 * - comp level 3:    45s, 639M
 * - comp level 4:    48s, 590M  (47% speedup vs default, <1% size increase)
 * - comp level 5:    61s, 589M
 * - libpng default:  98s, 586M
 * - comp level 9:   328s, 580M
 *
 * @todo investigate other zlib options
 */
static constexpr int DEFAULT_PNG_OSF_ZLIB_COMPRESSION_LEVEL = 1;

class OUSTER_API_CLASS PngLidarScanEncoder
    : public ouster::osf::LidarScanEncoder {
   public:
    OUSTER_API_FUNCTION
    PngLidarScanEncoder(int compression_amount)
        : compression_amount_{compression_amount} {}

    // This method encodes a field, if px_offset is provided it is destaggered
    // before encoding
    // FIXME[tws] method should be private, but "friend class/FRIEND_TEST" for
    // the unit test isn't working for some reason
    OUSTER_API_IGNORE
    ScanChannelData encodeField(
        const ouster::Field& field,
        const std::vector<int>& px_offset = {}) const override;

   private:
    int compression_amount_{DEFAULT_PNG_OSF_ZLIB_COMPRESSION_LEVEL};

    /**
     * @defgroup OSFPngEncode8 Encoding Functionality.
     * Encode img object into a 8 bit, Gray, PNG buffer.
     *
     * @tparam T The type to use for the output array.
     *
     * @param[out] res_buf The output buffer with a single encoded PNG.
     * @param[in] img The image object to encode.
     * @return false (0) if operation is successful, true (1) if error occured
     */

    /**
     * @copydoc OSFPngEncode8
     * @param[in] px_offset Pixel shift per row used to destagger img data.
     */
    template <typename T>
    bool encode8bitImage(ScanChannelData& res_buf,
                         const Eigen::Ref<const img_t<T>>& img,
                         const std::vector<int>& px_offset) const;

    /** @copydoc OSFPngEncode8 */
    template <typename T>
    bool encode8bitImage(ScanChannelData& res_buf,
                         const Eigen::Ref<const img_t<T>>& img) const;

    /**
     * Encode img object into a 16 bit, Gray, PNG buffer.
     *
     * @tparam T The type to use for the output array.
     *
     * @param[out] res_buf The output buffer with a single encoded PNG.
     * @param[in] img The image object to encode.
     * @param[in] px_offset Pixel shift per row used to destagger img data.
     * @return false (0) if operation is successful, true (1) if error occured
     */
    template <typename T>
    bool encode16bitImage(ScanChannelData& res_buf,
                          const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& px_offset) const;

    /**
     * Encode 2D image of a typical lidar scan field channel into a 16 bit,
     * Gray, PNG buffer.
     *
     * @tparam T The type to use for the output array.
     *
     * @param[out] res_buf The output buffer with a single encoded PNG.
     * @param[in] img The image object to encode.
     * @return false (0) if operation is successful, true (1) if error occured
     */
    template <typename T>
    bool encode16bitImage(ScanChannelData& res_buf,
                          const Eigen::Ref<const img_t<T>>& img) const;

    /**
     * @defgroup OSFPngEncode32 Encoding Functionality.
     * Encode 2D image of a typical lidar scan field channel into a 32 bit,
     * RGBA, PNG buffer.
     *
     * @tparam T The type to use for the output array.
     *
     * @param[out] res_buf The output buffer with a single encoded PNG.
     * @param[in] img 2D image or a single LidarScan field data.
     * @return false (0) if operation is successful, true (1) if error occured
     */

    /**
     * @copydoc OSFPngEncode32
     * @param[in] px_offset Pixel shift per row used to destagger img data.
     */
    template <typename T>
    bool encode32bitImage(ScanChannelData& res_buf,
                          const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& px_offset) const;

    /** @copydoc OSFPngEncode32 */
    template <typename T>
    bool encode32bitImage(ScanChannelData& res_buf,
                          const Eigen::Ref<const img_t<T>>& img) const;

    /**
     * @defgroup OSFPngEncode24 Encoding Functionality.
     * Encode 2D image of a typical lidar scan field channel into a 24 bit, RGB,
     * PNG buffer.
     *
     * @tparam T The type to use for the output array.
     *
     * @param[out] res_buf The output buffer with a single encoded PNG.
     * @param[in] img 2D image or a single LidarScan field data.
     * @return false (0) if operation is successful, true (1) if error occured
     */

    /**
     * @copydoc OSFPngEncode24
     * @param[in] px_offset Pixel shift per row used to destagger img data.
     */
    template <typename T>
    bool encode24bitImage(ScanChannelData& res_buf,
                          const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& px_offset) const;

    /** @copydoc OSFPngEncode24 */
    template <typename T>
    bool encode24bitImage(ScanChannelData& res_buf,
                          const Eigen::Ref<const img_t<T>>& img) const;

    /**
     * @defgroup OSFPngEncode64 Encoding Functionality.
     * Encode 2D image of a typical lidar scan field channel into a 64 bit,
     * RGBA, PNG buffer.
     *
     * @tparam T The type to use for the output array.
     *
     * @param[out] res_buf The output buffer with a single encoded PNG.
     * @param[in] img 2D image or a single LidarScan field data.
     * @return false (0) if operation is successful, true (1) if error occured
     */

    /**
     * @copydoc OSFPngEncode64
     * @param[in] px_offset Pixel shift per row used to destagger img data.
     */
    template <typename T>
    bool encode64bitImage(ScanChannelData& res_buf,
                          const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& px_offset) const;

    /** @copydoc OSFPngEncode64 */
    template <typename T>
    bool encode64bitImage(ScanChannelData& res_buf,
                          const Eigen::Ref<const img_t<T>>& img) const;
};
}  // namespace osf
}  // namespace ouster
