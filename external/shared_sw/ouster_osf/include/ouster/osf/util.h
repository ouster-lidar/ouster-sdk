#pragma once

#include <Eigen/Eigen>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/osf/common.h"
#include "ouster/osf/file_info.h"
#include "ouster/osf/types.h"
#include "ouster/osf/version.h"

namespace ouster {
namespace OSF {

// TODO[pb]: Hide in util_impl.h once GLAM switch to OSF Reader API.
static constexpr size_t SIZE_OF_PREFIXED_SIZE = 4;

// Sorted Window reader copnstant, optimal for current OSF streams
// and ensures that all messages read without loses (used in rechunk())
static constexpr int SORT_WINDOW_SIZE_LIDAR_SCAN = 200;
static constexpr int SORT_WINDOW_SIZE_TAJECTORY = 400;
static constexpr int SORT_WINDOW_SIZE_IMU = 2000;
static constexpr int SORT_WINDOW_SIZE_GPS = 100;

struct OsfBufferOpener {
    uint8_t* osf_file;
    size_t file_size;
    size_t chunks_offset;
    size_t session_offset;
    OSF::OSF_VERSION version;
};

/**
 * File manipulation functions
 */

size_t initEmptyOsfFile(std::string filename);

void closeOsfFile(std::string filename, size_t session_offset,
                  size_t session_size);

OsfBufferOpener openOsfBuffer(const std::string& file_path);

void saveFlatBuffer(char* buf, int size, std::string filename,
                    bool append = false);

void savePrefixedSizeFlatBuffer(char* buf, uint32_t size, std::string filename,
                                bool append = false);

// TODO[pb]: Remove from public interface
size_t readPrefixedSizeFromOffset(const uint8_t* osf_file, size_t offset = 0);

// Parse OSF_FRAME_MODE from string
OSF::OSF_FRAME_MODE osf_frame_mode_of_string(const std::string& frame_mode);

// Return string representation of frame_mode
std::string to_string(const OSF_FRAME_MODE frame_mode);

/**
 * High-level operations powering cli utils
 */

/**
 * @param file non-chunked osf file path
 * @param dest_file path of chunked osf file to write
 * @param frame_mode output OSF lidar scan encoding. Recode from input
 * OSF file frame_mode to the specified output if they differ.
 * @param use_car_trajectory if set - interpolate the sensors per column
 * trajectories from car trajectory. If car trajectory doesn't exist it
 * fallbacks and uses the existing per sensor trajectories). If not set - use
 * only existing per sensor trajectories from input OSF.
 * default - use car trajectory
 */
void rechunk(std::string file, std::string dest_file,
             const OSF::OSF_FRAME_MODE output_frame_mode =
                 OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_32,
             const bool use_car_trajectory = true);

/**
 * @param file chunked osf file path
 * @param dest_dir directory to write legacy osf pngs and jsons
 */
void stream2LegacyOsf(const std::string& file, const std::string& dest_dir);


/**
 * Calculates avg error between two trajectories of equal size.
 *
 * @return tuple<rotation_error, translation_error>
 */
std::pair<double, double> calc_traj_error(OSF::Trajectory& gt_traj,
                                          OSF::Trajectory& traj);

}  // namespace OSF
}  // namespace ouster
