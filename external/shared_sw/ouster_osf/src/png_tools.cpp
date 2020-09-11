#include "png_tools.h"

#include <png.h>

#include <Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <memory>

#include "ouster/lidar_scan.h"
#include "util_impl.h"

namespace ouster {
namespace OSF {

/*
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
 * TODO: investigate other zlib options
 */
static constexpr int PNG_OSF_ZLIB_COMPRESSION_LEVEL = 4;

/**
 * Provides the data reader capabilities from std::vector for png_read IO
 */
struct VectorReader {
    const std::vector<uint8_t>& buffer;
    size_t offset;
    explicit VectorReader(const std::vector<uint8_t>& buf)
        : buffer(buf), offset(0) {}
    void read(void* bytes, const size_t bytes_len) {
        // Skip safety check and trust libpng?
        if (offset >= buffer.size()) return;
        size_t bytes_to_read = bytes_len;
        if (offset + bytes_to_read > buffer.size()) {
            bytes_to_read = buffer.size() - offset;
        }
        std::memcpy(bytes, buffer.data() + offset, bytes_to_read);
        offset += bytes_to_read;
    }
};

/**
 * Error callback that will be fired on libpng errors
 */
void png_osf_error(png_structp png_ptr, png_const_charp msg) {
    std::cout << "ERROR libpng osf: " << msg << std::endl;
    longjmp(png_jmpbuf(png_ptr), 1);
};

/**
 * Custom png_write handler to write data to std::vector buffer
 */
void png_osf_write_data(png_structp png_ptr, png_bytep bytes,
                        png_size_t bytes_len) {
    std::vector<uint8_t>* res_buf =
        reinterpret_cast<std::vector<uint8_t>*>(png_get_io_ptr(png_ptr));
    res_buf->insert(res_buf->end(), reinterpret_cast<uint8_t*>(bytes),
                    reinterpret_cast<uint8_t*>(bytes + bytes_len));
};

/**
 * Custom png_read handler to read data from std::vector (via VectorReader
 * helper)
 */
void png_osf_read_data(png_structp png_ptr, png_bytep bytes,
                       png_size_t bytes_len) {
    VectorReader* vec_read =
        reinterpret_cast<VectorReader*>(png_get_io_ptr(png_ptr));
    vec_read->read(bytes, bytes_len);
};

// void user_read_data(png_structp png_ptr, png_bytep data, png_size_t length);

/**
 * It's needed for custom png IO operations... but I've never seen it's called.
 * And also there are no need to flush writer to std::vector buufer in our case.
 */
void png_osf_flush_data(png_structp){};

/**
 * Common png WRITE init routine, creates and setups png_ptr and png_info_ptr
 */
bool png_osf_write_init(png_structpp png_ptrp, png_infopp png_info_ptrp) {
    *png_ptrp = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                        png_osf_error, png_osf_error);
    if (!*png_ptrp) {
        std::cout << "ERROR: no png_ptr\n";
        return true;
    }

    *png_info_ptrp = png_create_info_struct(*png_ptrp);
    if (!*png_info_ptrp) {
        std::cout << "ERROR: no png_info_ptr\n";
        png_destroy_write_struct(png_ptrp, nullptr);
        return true;
    }

    return false;  // SUCCESS
}

/**
 * Common png READ init routine, creates and setups png_ptr and png_info_ptr
 */
bool png_osf_read_init(png_structpp png_ptrp, png_infopp png_info_ptrp) {
    *png_ptrp = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr,
                                       png_osf_error, png_osf_error);
    if (!*png_ptrp) {
        std::cout << "ERROR: no png_ptr\n";
        return true;
    }

    *png_info_ptrp = png_create_info_struct(*png_ptrp);
    if (!*png_info_ptrp) {
        std::cout << "ERROR: no png_info_ptr\n";
        png_destroy_read_struct(png_ptrp, nullptr, nullptr);
        return true;
    }

    return false;  // SUCCESS
}

/**
 * Common png write setup routine.
 * Write destination is res_buf of std::vector<uitn8_t> type.
 */
void png_osf_write_start(png_structp png_ptr, png_infop png_info_ptr,
                         ScanChannelData& res_buf, const uint32_t width,
                         const uint32_t height, const int sample_depth,
                         const int color_type) {
    // Use setjmp() on upper level for errors catching
    png_set_write_fn(png_ptr, &res_buf, png_osf_write_data, png_osf_flush_data);

    png_set_compression_level(png_ptr, PNG_OSF_ZLIB_COMPRESSION_LEVEL);

    png_set_IHDR(png_ptr, png_info_ptr, width, height, sample_depth, color_type,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);

    png_write_info(png_ptr, png_info_ptr);
}

// ========== Encode Functions ===================================

ScanData scanEncodeOSF32(const LidarScan& lidar_scan,
                         const std::vector<int>& px_offset,
                         const int range_multiplier) {
    ScanData scan_data(1);
    scanEncodeOSF32Channel(scan_data[0], lidar_scan, px_offset,
                           range_multiplier);
    return scan_data;
}

ScanData scanEncodeOSF56(const LidarScan& lidar_scan,
                         const std::vector<int>& px_offset) {
    ScanData scan_data(3);
    scanEncode24bitChannel(scan_data[0], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::RANGE);
    scanEncode16bitChannel(scan_data[1], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::INTENSITY);
    scanEncode16bitChannel(scan_data[2], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::NOISE);
    return scan_data;
}

ScanData scanEncodeOSF40RI(const LidarScan& lidar_scan,
                           const std::vector<int>& px_offset) {
    ScanData scan_data(2);
    scanEncode24bitChannel(scan_data[0], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::RANGE);
    scanEncode16bitChannel(scan_data[1], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::INTENSITY);
    return scan_data;
}

ScanData scanEncodeOSF40RN(const LidarScan& lidar_scan,
                           const std::vector<int>& px_offset) {
    ScanData scan_data(2);
    scanEncode24bitChannel(scan_data[0], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::RANGE);
    scanEncode16bitChannel(scan_data[1], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::NOISE);
    return scan_data;
}

ScanData scanEncodeOSF72(const LidarScan& lidar_scan,
                         const std::vector<int>& px_offset) {
    ScanData scan_data(4);
    scanEncode24bitChannel(scan_data[0], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::RANGE);
    scanEncode16bitChannel(scan_data[1], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::INTENSITY);
    scanEncode16bitChannel(scan_data[2], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::NOISE);
    scanEncode16bitChannel(scan_data[3], lidar_scan, px_offset,
                           LidarScan::LidarScanIndex::REFLECTIVITY);
    return scan_data;
}

// libpng pnly write version
bool scanEncodeOSF32Channel(ScanChannelData& res_buf,
                            const LidarScan& lidar_scan,
                            const std::vector<int>& px_offset,
                            const int range_multiplier) {
    const size_t width = lidar_scan.w;
    const size_t height = lidar_scan.h;

    // OSF_32 Params
    const int sample_depth = 8;
    const int color_type = PNG_COLOR_TYPE_RGB_ALPHA;

    // OSF_32 Encoding Sizes
    std::vector<uint8_t> row_data(width * 4);  // RGBA

    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_write_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &png_info_ptr);
        return true;
    }

    png_osf_write_start(png_ptr, png_info_ptr, res_buf, width, height,
                        sample_depth, color_type);

    // Destagger fields
    const LidarScan::field_t<LidarScan::raw_t> range_d =
        destagger<LidarScan::raw_t>(lidar_scan.range(), px_offset);
    const LidarScan::field_t<LidarScan::raw_t> intensity_d =
        destagger<LidarScan::raw_t>(lidar_scan.intensity(), px_offset);
    const LidarScan::field_t<LidarScan::raw_t> noise_d =
        destagger<LidarScan::raw_t>(lidar_scan.noise(), px_offset);

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            const size_t index = lidar_scan.ind(u, v);
            const uint32_t range_val = range_d(index);
            const uint32_t intensity_val = intensity_d(index);
            const uint32_t noise_val = noise_d(index);

            // OSF_32 Encoding Logic
            row_data[v * 4 + 0] =
                static_cast<uint8_t>((range_val / range_multiplier) & 0xff);
            row_data[v * 4 + 1] = static_cast<uint8_t>(
                ((range_val / range_multiplier) >> 8u) & 0xff);
            row_data[v * 4 + 2] =
                static_cast<uint8_t>(std::min(noise_val, 255u));
            row_data[v * 4 + 3] =
                static_cast<uint8_t>(std::min(intensity_val, 255u));
        }

        png_write_row(png_ptr,
                      reinterpret_cast<png_const_bytep>(row_data.data()));
    }

    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &png_info_ptr);

    return false;  // SUCCESS
}

bool scanEncode16bitChannel(ScanChannelData& res_buf,
                            const LidarScan& lidar_scan,
                            const std::vector<int>& px_offset,
                            const LidarScan::LidarScanIndex channel_index) {
    const size_t width = lidar_scan.w;
    const size_t height = lidar_scan.h;

    // 16 bit Gray
    const int sample_depth = 16;
    const int color_type = PNG_COLOR_TYPE_GRAY;

    // 16bit Encoding Sizes
    std::vector<uint8_t> row_data(width * 2);  // Gray, 16bit

    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_write_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &png_info_ptr);
        return true;
    }

    png_osf_write_start(png_ptr, png_info_ptr, res_buf, width, height,
                        sample_depth, color_type);

    // Needed to transform provided little-endian samples to internal
    // PNG big endian format
    png_set_swap(png_ptr);

    // Get destaggered lidar scan channel
    const LidarScan::field_t<LidarScan::raw_t> channel_key_d =
        destagger<LidarScan::raw_t>(lidar_scan.data.col(channel_index),
                                    px_offset);

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            const size_t index = lidar_scan.ind(u, v);
            const uint32_t key_val = channel_key_d(index);

            // 16bit Encoding Logic
            row_data[v * 2] = static_cast<uint8_t>(key_val & 0xff);
            row_data[v * 2 + 1] = static_cast<uint8_t>((key_val >> 8u) & 0xff);
        }

        png_write_row(png_ptr,
                      reinterpret_cast<png_const_bytep>(row_data.data()));
    }

    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &png_info_ptr);

    return false;  // SUCCESS
}

bool scanEncode24bitChannel(ScanChannelData& res_buf,
                            const LidarScan& lidar_scan,
                            const std::vector<int>& px_offset,
                            const LidarScan::LidarScanIndex channel_index) {
    const size_t width = lidar_scan.w;
    const size_t height = lidar_scan.h;

    // 8bit RGB
    const int sample_depth = 8;
    const int color_type = PNG_COLOR_TYPE_RGB;

    // 24bit Encoding Sizes
    std::vector<uint8_t> row_data(width * 3);  // RGB, 8bit

    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_write_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &png_info_ptr);
        return true;
    }

    png_osf_write_start(png_ptr, png_info_ptr, res_buf, width, height,
                        sample_depth, color_type);

    // Get destaggered lidar scan channel
    const LidarScan::field_t<LidarScan::raw_t> channel_key_d =
        destagger<LidarScan::raw_t>(lidar_scan.data.col(channel_index),
                                    px_offset);

    for (size_t u = 0; u < height; ++u) {
        for (size_t v = 0; v < width; ++v) {
            const size_t index = lidar_scan.ind(u, v);
            const uint32_t key_val = channel_key_d(index);

            // 24bit Encoding Logic
            row_data[v * 3 + 0] = static_cast<uint8_t>(key_val & 0xff);
            row_data[v * 3 + 1] = static_cast<uint8_t>((key_val >> 8u) & 0xff);
            row_data[v * 3 + 2] = static_cast<uint8_t>((key_val >> 16u) & 0xff);
        }

        png_write_row(png_ptr,
                      reinterpret_cast<png_const_bytep>(row_data.data()));
    }

    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &png_info_ptr);

    return false;  // SUCCESS
}

ScanData scanEncode(const LidarScan& lidar_scan,
                    const std::vector<int>& px_offset,
                    const OSF_FRAME_MODE frame_mode,
                    const int range_multiplier) {
    switch (frame_mode) {
        case ouster::OSF::OSF_FRAME_MODE_OSF_32:
            return scanEncodeOSF32(lidar_scan, px_offset, range_multiplier);

        case ouster::OSF::OSF_FRAME_MODE_OSF_40RI:
            return scanEncodeOSF40RI(lidar_scan, px_offset);

        case ouster::OSF::OSF_FRAME_MODE_OSF_40RN:
            return scanEncodeOSF40RN(lidar_scan, px_offset);

        case ouster::OSF::OSF_FRAME_MODE_OSF_56:
            return scanEncodeOSF56(lidar_scan, px_offset);

        case ouster::OSF::OSF_FRAME_MODE_OSF_72:
            return scanEncodeOSF72(lidar_scan, px_offset);

        default:
            std::cerr << "OSF_FRAME_MODE not supported in current version"
                      << std::endl;
            return {};  // ERROR
    }
}

// ========== Decode Functions ===================================

bool scanDecode(LidarScan& lidar_scan, const ScanData& scan_data,
                const std::vector<int>& px_offset,
                const OSF_FRAME_MODE frame_mode, const int range_multiplier) {
    switch (frame_mode) {
        case ouster::OSF::OSF_FRAME_MODE_OSF_32:
            return scanDecodeOSF32(lidar_scan, scan_data, px_offset,
                                   range_multiplier);

        case ouster::OSF::OSF_FRAME_MODE_OSF_40RI:
            return scanDecodeOSF40RI(lidar_scan, scan_data, px_offset);

        case ouster::OSF::OSF_FRAME_MODE_OSF_40RN:
            return scanDecodeOSF40RN(lidar_scan, scan_data, px_offset);

        case ouster::OSF::OSF_FRAME_MODE_OSF_56:
            return scanDecodeOSF56(lidar_scan, scan_data, px_offset);

        case ouster::OSF::OSF_FRAME_MODE_OSF_72:
            return scanDecodeOSF72(lidar_scan, scan_data, px_offset);

        default:
            std::cerr << "OSF_FRAME_MODE == "
                      << EnumNamesOSF_FRAME_MODE()[frame_mode]
                      << " is not supported in current version" << std::endl;
            return true;  // ERROR
    }
}

bool scanDecodeOSF32(LidarScan& lidar_scan, const ScanData& scan_data,
                     const std::vector<int>& px_offset,
                     const int range_multiplier) {
    // Should have one scan channel
    if (scan_data.size() != 1) {
        std::cout << "ERROR: lidar_scan data contains # of channels: "
                  << scan_data.size() << ", expected: 1 for OSF_32"
                  << std::endl;
        return true;
    }

    if (scanDecodeOSF32Channel(lidar_scan, scan_data[0], px_offset,
                               range_multiplier)) {
        return true;
    }

    // Reflectivity is not present in OSF_32 so need to zero it out
    lidar_scan.reflectivity().setZero();

    return false;  // SUCCESS
}

bool scanDecodeOSF40RI(LidarScan& lidar_scan, const ScanData& scan_data,
                       const std::vector<int>& px_offset) {
    // Should have three scan channels
    if (scan_data.size() != 2) {
        std::cout << "ERROR: lidar_scan data contains # of channels: "
                  << scan_data.size() << ", expected: 2 for OSF_40RI"
                  << std::endl;
        return true;
    }

    if (scanDecode24bitChannel(lidar_scan, scan_data[0], px_offset,
                               LidarScan::LidarScanIndex::RANGE)) {
        return true;
    }

    if (scanDecode16bitChannel(lidar_scan, scan_data[1], px_offset,
                               LidarScan::LidarScanIndex::INTENSITY)) {
        return true;
    }

    // Noise is not present in OSF_40RI so need to zero it out
    lidar_scan.noise().setZero();

    // Reflectivity is not present in OSF_40RI so need to zero it out
    lidar_scan.reflectivity().setZero();

    return false;  // SUCCESS
}

bool scanDecodeOSF40RN(LidarScan& lidar_scan, const ScanData& scan_data,
                       const std::vector<int>& px_offset) {
    // Should have three scan channels
    if (scan_data.size() != 2) {
        std::cout << "ERROR: lidar_scan data contains # of channels: "
                  << scan_data.size() << ", expected: 2 for OSF_40RN"
                  << std::endl;
        return true;
    }

    if (scanDecode24bitChannel(lidar_scan, scan_data[0], px_offset,
                               LidarScan::LidarScanIndex::RANGE)) {
        return true;
    }

    if (scanDecode16bitChannel(lidar_scan, scan_data[1], px_offset,
                               LidarScan::LidarScanIndex::NOISE)) {
        return true;
    }

    // Intensity is not present in OSF_40RN so need to zero it out
    lidar_scan.intensity().setZero();

    // Reflectivity is not present in OSF_40RI so need to zero it out
    lidar_scan.reflectivity().setZero();

    return false;  // SUCCESS
}

bool scanDecodeOSF56(LidarScan& lidar_scan, const ScanData& scan_data,
                     const std::vector<int>& px_offset) {
    // Should have three scan channels
    if (scan_data.size() != 3) {
        std::cout << "ERROR: lidar_scan data contains # of channels: "
                  << scan_data.size() << ", expected: 3 for OSF_56"
                  << std::endl;
        return true;
    }

    if (scanDecode24bitChannel(lidar_scan, scan_data[0], px_offset,
                               LidarScan::LidarScanIndex::RANGE)) {
        return true;
    }

    if (scanDecode16bitChannel(lidar_scan, scan_data[1], px_offset,
                               LidarScan::LidarScanIndex::INTENSITY)) {
        return true;
    }

    if (scanDecode16bitChannel(lidar_scan, scan_data[2], px_offset,
                               LidarScan::LidarScanIndex::NOISE)) {
        return true;
    }

    // Reflectivity is not present in OSF_56 so need to zero it out
    lidar_scan.reflectivity().setZero();

    return false;  // SUCCESS
}

bool scanDecodeOSF72(LidarScan& lidar_scan, const ScanData& scan_data,
                     const std::vector<int>& px_offset) {
    // Should have three scan channels
    if (scan_data.size() != 4) {
        std::cout << "ERROR: lidar_scan data contains # of channels: "
                  << scan_data.size() << ", expected: 4 for OSF_72"
                  << std::endl;
        return true;
    }

    if (scanDecode24bitChannel(lidar_scan, scan_data[0], px_offset,
                               LidarScan::LidarScanIndex::RANGE)) {
        return true;
    }

    if (scanDecode16bitChannel(lidar_scan, scan_data[1], px_offset,
                               LidarScan::LidarScanIndex::INTENSITY)) {
        return true;
    }

    if (scanDecode16bitChannel(lidar_scan, scan_data[2], px_offset,
                               LidarScan::LidarScanIndex::NOISE)) {
        return true;
    }

    if (scanDecode16bitChannel(lidar_scan, scan_data[3], px_offset,
                               LidarScan::LidarScanIndex::REFLECTIVITY)) {
        return true;
    }

    return false;  // SUCCESS
}

bool scanDecodeOSF32Channel(LidarScan& lidar_scan,
                            const ScanChannelData& channel_buf,
                            const std::vector<int>& px_offset,
                            const int range_multiplier) {
    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_read_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);
        return true;
    }

    VectorReader channel_reader(channel_buf);
    png_set_read_fn(png_ptr, &channel_reader, png_osf_read_data);

    // only used with Gray 16 bit to get little-endian LSB representation back
    // for other color types PNG_TRANSFORM_SWAP_ENDIAN does nothing
    int transforms = PNG_TRANSFORM_SWAP_ENDIAN;
    png_read_png(png_ptr, png_info_ptr, transforms, nullptr);

    png_uint_32 width;
    png_uint_32 height;
    int sample_depth;
    int color_type;

    png_get_IHDR(png_ptr, png_info_ptr, &width, &height, &sample_depth,
                 &color_type, nullptr, nullptr, nullptr);

    png_bytepp row_pointers = png_get_rows(png_ptr, png_info_ptr);

    // Sanity checks for encoded PNG size
    if (width != lidar_scan.w || height != lidar_scan.h) {
        std::cout
            << "ERROR: lidar_scan chanel contains data of incompatible size: "
            << width << "x" << height << ", expected: " << lidar_scan.w << "x"
            << lidar_scan.h << std::endl;
        return true;
    }

    if (sample_depth != 8) {
        std::cout << "ERROR: lidar_scan chanel contains data with incompatible "
                     "sample_depth: "
                  << sample_depth << ", expected: 8" << std::endl;
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_RGBA) {
        std::cout << "ERROR: lidar_scan chanel contains data with incompatible "
                     "color type: "
                  << color_type << ", expected: " << PNG_COLOR_TYPE_RGBA
                  << std::endl;
        return true;
    }

    // OSF_32 Decoding to LidarScan
    for (size_t u = 0; u < height; u++) {
        for (size_t v = 0; v < width; v++) {
            const LidarScan::index_t index = lidar_scan.ind(u, v);

            lidar_scan.range()(index) =
                range_multiplier * (row_pointers[u][v * 4 + 0] +
                                    (row_pointers[u][v * 4 + 1] << 8u));

            lidar_scan.noise()(index) = row_pointers[u][v * 4 + 2];
            lidar_scan.intensity()(index) = row_pointers[u][v * 4 + 3];
        }
    }

    // Make a default staggered representation of the fields
    lidar_scan.range() =
        destagger<LidarScan::raw_t, -1>(lidar_scan.range(), px_offset);
    lidar_scan.noise() =
        destagger<LidarScan::raw_t, -1>(lidar_scan.noise(), px_offset);
    lidar_scan.intensity() =
        destagger<LidarScan::raw_t, -1>(lidar_scan.intensity(), px_offset);

    png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);

    return false;  // SUCCESS
}

bool scanDecode24bitChannel(LidarScan& lidar_scan,
                            const ScanChannelData& channel_buf,
                            const std::vector<int>& px_offset,
                            const LidarScan::LidarScanIndex channel_index) {
    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_read_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);
        return true;
    }

    VectorReader channel_reader(channel_buf);
    png_set_read_fn(png_ptr, &channel_reader, png_osf_read_data);

    // only used with Gray 16 bit to get little-endian LSB representation back
    // for other color types PNG_TRANSFORM_SWAP_ENDIAN does nothing
    int transforms = PNG_TRANSFORM_SWAP_ENDIAN;
    png_read_png(png_ptr, png_info_ptr, transforms, nullptr);

    png_uint_32 width;
    png_uint_32 height;
    int sample_depth;
    int color_type;

    png_get_IHDR(png_ptr, png_info_ptr, &width, &height, &sample_depth,
                 &color_type, nullptr, nullptr, nullptr);

    png_bytepp row_pointers = png_get_rows(png_ptr, png_info_ptr);

    // Sanity checks for encoded PNG size
    if (width != lidar_scan.w || height != lidar_scan.h) {
        std::cout
            << "ERROR: lidar_scan chanel contains data of incompatible size: "
            << width << "x" << height << ", expected: " << lidar_scan.w << "x"
            << lidar_scan.h << std::endl;
        return true;
    }

    if (sample_depth != 8) {
        std::cout << "ERROR: lidar_scan chanel contains data with incompatible "
                     "sample_depth: "
                  << sample_depth << ", expected: 8" << std::endl;
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_RGB) {
        std::cout << "ERROR: lidar_scan chanel contains data with incompatible "
                     "color type: "
                  << color_type << ", expected: " << PNG_COLOR_TYPE_RGB
                  << std::endl;
        return true;
    }

    LidarScan::data_t::ColXpr channel_key = lidar_scan.data.col(channel_index);

    // 24bit channel data decoding to LidarScan for key channel_index
    for (size_t u = 0; u < height; u++) {
        for (size_t v = 0; v < width; v++) {
            const LidarScan::index_t index = lidar_scan.ind(u, v);

            channel_key(index) = row_pointers[u][v * 3 + 0] +
                                 (row_pointers[u][v * 3 + 1] << 8u) +
                                 (row_pointers[u][v * 3 + 2] << 16u);
        }
    }

    // Make a default staggered representation of a field
    lidar_scan.data.col(channel_index) = destagger<LidarScan::raw_t, -1>(
        lidar_scan.data.col(channel_index), px_offset);

    png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);

    return false;  // SUCCESS
}

bool scanDecode16bitChannel(LidarScan& lidar_scan,
                            const ScanChannelData& channel_buf,
                            const std::vector<int>& px_offset,
                            const LidarScan::LidarScanIndex channel_index) {
    // libpng main structs
    png_structp png_ptr;
    png_infop png_info_ptr;

    if (png_osf_read_init(&png_ptr, &png_info_ptr)) {
        return true;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);
        return true;
    }

    VectorReader channel_reader(channel_buf);
    png_set_read_fn(png_ptr, &channel_reader, png_osf_read_data);

    // only used with Gray 16 bit to get little-endian LSB representation back
    // for other color types PNG_TRANSFORM_SWAP_ENDIAN does nothing
    int transforms = PNG_TRANSFORM_SWAP_ENDIAN;
    png_read_png(png_ptr, png_info_ptr, transforms, nullptr);

    png_uint_32 width;
    png_uint_32 height;
    int sample_depth;
    int color_type;

    png_get_IHDR(png_ptr, png_info_ptr, &width, &height, &sample_depth,
                 &color_type, nullptr, nullptr, nullptr);

    png_bytepp row_pointers = png_get_rows(png_ptr, png_info_ptr);

    // Sanity checks for encoded PNG size
    if (width != lidar_scan.w || height != lidar_scan.h) {
        std::cout
            << "ERROR: lidar_scan chanel contains data of incompatible size: "
            << width << "x" << height << ", expected: " << lidar_scan.w << "x"
            << lidar_scan.h << std::endl;
        return true;
    }

    if (sample_depth != 16) {
        std::cout << "ERROR: lidar_scan chanel contains data with incompatible "
                     "sample_depth: "
                  << sample_depth << ", expected: 16" << std::endl;
        return true;
    }

    if (color_type != PNG_COLOR_TYPE_GRAY) {
        std::cout << "ERROR: lidar_scan chanel contains data with incompatible "
                     "color type: "
                  << color_type << ", expected: " << PNG_COLOR_TYPE_GRAY
                  << std::endl;
        return true;
    }

    LidarScan::data_t::ColXpr channel_key = lidar_scan.data.col(channel_index);

    // 16bit channel data decoding to LidarScan for key channel_index
    for (size_t u = 0; u < height; u++) {
        for (size_t v = 0; v < width; v++) {
            const LidarScan::index_t index = lidar_scan.ind(u, v);

            channel_key(index) =
                row_pointers[u][v * 2 + 0] + (row_pointers[u][v * 2 + 1] << 8u);
        }
    }

    // Make a default staggered representation of a field
    lidar_scan.data.col(channel_index) = destagger<LidarScan::raw_t, -1>(
        lidar_scan.data.col(channel_index), px_offset);

    png_destroy_read_struct(&png_ptr, &png_info_ptr, nullptr);

    return false;  // SUCCESS
}

// =================== Save to File Functions ====================

bool saveScanChannel(const ScanChannelData& channel_buf,
                     const std::string& filename) {
    std::fstream file(filename, std::ios_base::out | std::ios_base::binary);

    if (file.good()) {
        file.write(reinterpret_cast<const char*>(channel_buf.data()),
                   channel_buf.size());
        if (file.good()) {
            file.close();
            return false;  // SUCCESS
        }
    }

    file.close();
    return true;  // FAILURE
}

}  // namespace OSF
}  // namespace ouster