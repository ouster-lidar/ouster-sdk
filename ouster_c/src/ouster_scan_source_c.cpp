// C scan source wrapper implementation
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "ouster/cartesian.h"           // cartesian, XYZLut, make_xyz_lut
#include "ouster/sensor_scan_source.h"  // SensorScanSource
#include "ouster/types.h"               // sensor_info
#include "ouster_c.h"

struct ouster_scan_source {
    std::unique_ptr<ouster::sensor::SensorScanSource> source;
};

struct ouster_lidar_scan {
    std::unique_ptr<ouster::LidarScan> ls;
};

struct ouster_xyz_lut {
    std::unique_ptr<ouster::XYZLut> lut;
};

extern "C" {

/* ================= Scan Source Implementation ================= */

int ouster_scan_source_create(const char* hostname,
                              ouster_scan_source_t** out_source) {
    if (!hostname || !out_source) return -1;
    try {
        std::vector<ouster::sensor::Sensor> sensors;
        ouster::sensor::sensor_config cfg;  // default
        cfg.udp_dest = "@auto";             // auto-detect
        sensors.emplace_back(std::string(hostname), cfg);
        auto src_ptr = std::make_unique<ouster_scan_source>();
        src_ptr->source =
            std::make_unique<ouster::sensor::SensorScanSource>(sensors);
        *out_source = src_ptr.release();
        return 0;
    } catch (...) {
        return -2;
    }
}

void ouster_scan_source_destroy(ouster_scan_source_t* source) {
    if (!source) return;

    source->source.reset();
    delete source;
}

int ouster_scan_source_frame_dimensions(const ouster_scan_source_t* source,
                                        int* width, int* height) {
    if (!source || source->source->sensor_info().empty()) return -1;

    auto& si = *source->source->sensor_info()[0];
    if (width) *width = (int)si.format.columns_per_frame;
    if (height) *height = (int)si.format.pixels_per_column;
    return 0;
}

int ouster_scan_source_get_metadata(ouster_scan_source_t* source, char* buffer,
                                    size_t capacity) {
    if (!source || source->source->sensor_info().empty()) return -1;

    const std::string& md = source->source->sensor_info()[0]->to_json_string();
    if (!buffer || capacity == 0) return (int)md.size();

    size_t to_copy = md.size() < capacity ? md.size() : capacity - 1;
    std::memcpy(buffer, md.data(), to_copy);
    buffer[to_copy] = '\0';
    return (int)to_copy;
}

/* ================= LidarScan Implementation ================= */

ouster_lidar_scan_t* ouster_scan_source_next_scan(ouster_scan_source_t* source,
                                                  int timeout_sec) {
    if (!source || !source->source) return nullptr;
    auto start = std::chrono::steady_clock::now();
    while (true) {
        auto result = source->source->get_scan();
        if (result.second) {
            ouster_lidar_scan_t* scan_handle =
                new (std::nothrow) ouster_lidar_scan();
            if (!scan_handle) return nullptr;
            scan_handle->ls = std::move(result.second);
            return scan_handle;
        }
        if (timeout_sec >= 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                               std::chrono::steady_clock::now() - start)
                               .count();
            if (elapsed >= timeout_sec) return nullptr;  // timeout
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void ouster_lidar_scan_destroy(ouster_lidar_scan_t* scan) {
    if (!scan) return;
    scan->ls.reset();
    delete scan;
}

void ouster_lidar_scan_get_dimensions(const ouster_lidar_scan_t* scan,
                                      int* width, int* height) {
    if (!scan || !scan->ls) return;
    if (width) *width = static_cast<int>(scan->ls->w);
    if (height) *height = static_cast<int>(scan->ls->h);
}

int ouster_lidar_scan_get_field_u32(const ouster_lidar_scan_t* scan,
                                    const char* field_name, uint32_t* out_buf,
                                    size_t capacity, size_t* out_count) {
    if (!scan || !scan->ls || !field_name || !out_buf) return -3;
    if (!scan->ls->has_field(field_name)) return -1;
    auto arr = scan->ls->field<uint32_t>(field_name);
    size_t n = (size_t)arr.size();
    if (capacity < n) return -2;
    std::memcpy(out_buf, arr.data(), n * sizeof(uint32_t));
    if (out_count) *out_count = n;
    return 0;
}

int ouster_lidar_scan_get_field_u16(const ouster_lidar_scan_t* scan,
                                    const char* field_name, uint16_t* out_buf,
                                    size_t capacity, size_t* out_count) {
    if (!scan || !scan->ls || !field_name || !out_buf) return -3;
    if (!scan->ls->has_field(field_name)) return -1;
    auto arr = scan->ls->field<uint16_t>(field_name);
    size_t n = (size_t)arr.size();
    if (capacity < n) return -2;
    std::memcpy(out_buf, arr.data(), n * sizeof(uint16_t));
    if (out_count) *out_count = n;
    return 0;
}

int ouster_lidar_scan_get_xyz(const ouster_lidar_scan_t* scan,
                              const ouster_xyz_lut_t* xyz_lut, float* xyz_out,
                              size_t capacity_points, size_t* out_points,
                              int filter_invalid) {
    if (!xyz_lut || !xyz_lut->lut || !scan || !scan->ls || !xyz_out) return -3;

    auto cloud = ouster::cartesian(*scan->ls, *xyz_lut->lut);
    size_t total = (size_t)cloud.rows();
    size_t written = 0;
    if (!filter_invalid && capacity_points < total) return -2;
    for (int i = 0; i < cloud.rows(); ++i) {
        auto xyz = cloud.row(i);
        bool is_zero = xyz.isApproxToConstant(0.0);
        if (filter_invalid && is_zero) continue;
        if (written >= capacity_points)
            break;  // stop if capacity filled when filtering
        xyz_out[3 * written + 0] = static_cast<float>(xyz(0));
        xyz_out[3 * written + 1] = static_cast<float>(xyz(1));
        xyz_out[3 * written + 2] = static_cast<float>(xyz(2));
        written++;
    }
    if (out_points) *out_points = written;
    return 0;
}

/* ===== Explicit XYZLut management ===== */

ouster_xyz_lut_t* ouster_scan_source_create_xyz_lut(
    const ouster_scan_source_t* source, int use_extrinsics) {
    if (!source || !source->source || source->source->sensor_info().empty())
        return nullptr;

    try {
        auto si = source->source->sensor_info()[0];
        auto lut_val = ouster::make_xyz_lut(*si, use_extrinsics != 0);
        ouster_xyz_lut_t* handle = new (std::nothrow) ouster_xyz_lut();
        if (!handle) return nullptr;
        handle->lut = std::make_unique<ouster::XYZLut>(std::move(lut_val));
        return handle;
    } catch (...) {
        return nullptr;
    }
}

void ouster_xyz_lut_destroy(ouster_xyz_lut_t* lut) {
    if (!lut) return;

    lut->lut.reset();
    delete lut;
}

}  // extern "C"
