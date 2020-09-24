#include "ouster/osf/util.h"

#include <fcntl.h>
#include <flatbuffers/flatbuffers.h>
#include <png.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iomanip>
#include <iostream>

#include "osfChunk_generated.h"
#include "osfHeader_generated.h"
#include "ouster/osf/lidar_scan_stat.h"
#include "png_tools.h"
#include "util_impl.h"

namespace ouster {
namespace OSF {

namespace os = ouster::sensor;

// Check that we have well formed chunk that is consistent with the
// current generated code. Chunks are always size prefixed.
bool isChunkValid(const uint8_t* buf) {
    size_t chunk_size = readPrefixedSizeFromOffset(buf);
    auto verifier = flatbuffers::Verifier(buf + 4,  // + 4 => size prefixed
                                          chunk_size);
    return VerifyosfChunkBuffer(verifier);
}

OsfBufferOpener openOsfBuffer(const std::string& file_path) {
    // std::cerr << "Reading " << file_path << std::endl;
    struct stat st;
    stat(file_path.c_str(), &st);
    size_t file_size = st.st_size;

    if (file_size == 0) {
        std::cerr << "Error reading file : " << file_path << ", empty file."
                  << std::endl;
        return OsfBufferOpener{nullptr, 0, 0, 0, ouster::OSF::V_INVALID};
    }
    auto fd = open(file_path.c_str(), O_RDONLY);

    auto map_osf_file = mmap(0, file_size, PROT_READ, MAP_SHARED, fd, 0);
    if (map_osf_file == MAP_FAILED) {
        close(fd);
        perror("Error mmapping the file");
        exit(EXIT_FAILURE);
    }
    uint8_t* osf_file = reinterpret_cast<uint8_t*>(map_osf_file);
    auto my_osfHeader = ouster::OSF::GetSizePrefixedosfHeader(osf_file);
    if (my_osfHeader->status() != ouster::OSF::HEADER_STATUS_VALID) {
        std::cerr << "Error invalid osf header in file : " << file_path
                  << std::endl;
        exit(EXIT_FAILURE);
    }
    if (file_size != my_osfHeader->file_length()) {
        std::cerr << "Error invalid osf_size in header in file : " << file_path
                  << std::endl;
        std::cerr << "file_size : " << file_size << std::endl;
        std::cerr << "file_size from header : " << my_osfHeader->file_length()
                  << std::endl;

        exit(EXIT_FAILURE);
    }

    size_t prefixed_header_size = readPrefixedSizeFromOffset(osf_file);

    size_t chunks_offset = prefixed_header_size + SIZE_OF_PREFIXED_SIZE;
    auto session_offset = my_osfHeader->session_offset();

    if (chunks_offset < session_offset &&
        !isChunkValid(osf_file + chunks_offset)) {
        std::cerr << "Error osfChunk is not valid. File : " << file_path
                  << std::endl;
        std::cerr << "Version from header : " << my_osfHeader->version()
                  << std::endl;
        std::cerr << "file_size from header : " << my_osfHeader->file_length()
                  << std::endl;
        exit(EXIT_FAILURE);
    }

    return OsfBufferOpener{osf_file, file_size, chunks_offset, session_offset,
                           (ouster::OSF::OSF_VERSION)my_osfHeader->version()};
}

void saveFlatBuffer(flatbuffers::FlatBufferBuilder& builder,
                    std::string filename, bool append) {
    // Get buffer and save to file
    char* buf = (char*)builder.GetBufferPointer();
    int size = builder.GetSize();

    std::fstream myfile;
    if (append)
        myfile.open(filename, std::fstream::out | std::fstream::app |
                                  std::fstream::binary);
    else {
        myfile.open(filename, std::fstream::out | std::fstream::trunc |
                                  std::fstream::binary);
    }

    if (myfile.is_open()) {
        myfile.write(buf, size);
        myfile.close();
    } else {
        std::cout << "fail to open " << filename << std::endl;
    }
}

void saveFlatBuffer(char* buf, int size, std::string filename, bool append) {
    std::fstream myfile;
    if (append)
        myfile.open(filename, std::fstream::out | std::fstream::app |
                                  std::fstream::binary);
    else {
        myfile.open(filename, std::fstream::out | std::fstream::trunc |
                                  std::fstream::binary);
    }

    if (myfile.is_open()) {
        myfile.write(buf, size);
        myfile.close();
    } else {
        std::cout << "fail to open " << filename << std::endl;
    }
}

void savePrefixedSizeFlatBuffer(char* buf, uint32_t size, std::string filename,
                                bool append) {
    std::fstream myfile;
    if (append)
        myfile.open(filename, std::fstream::out | std::fstream::app |
                                  std::fstream::binary);
    else {
        myfile.open(filename, std::fstream::out | std::fstream::trunc |
                                  std::fstream::binary);
    }

    if (myfile.is_open()) {
        myfile << (char)size << (char)(size >> 8u) << (char)(size >> 16u)
               << (char)(size >> 24u);
        myfile.write(buf, size);
        myfile.close();
    } else {
        std::cout << "fail to open " << filename << std::endl;
    }
}

size_t initEmptyOsfFile(std::string filename) {
    auto header_builder = flatbuffers::FlatBufferBuilder(1024);
    auto header = ouster::OSF::CreateosfHeader(
        header_builder, ouster::OSF::OSF_VERSION::V_1_4,
        ouster::OSF::HEADER_STATUS_INVALID, 0, 0);
    header_builder.FinishSizePrefixed(header,
                                      ouster::OSF::osfHeaderIdentifier());
    ouster::OSF::saveFlatBuffer(header_builder, filename, false);
    return header_builder.GetSize();
}

void closeOsfFile(std::string filename, size_t session_offset,
                  size_t session_size) {
    auto header_builder = flatbuffers::FlatBufferBuilder(1024);
    auto header = ouster::OSF::CreateosfHeader(
        header_builder, ouster::OSF::OSF_VERSION::V_1_4,
        ouster::OSF::HEADER_STATUS_VALID, session_offset,
        session_offset + session_size);
    header_builder.FinishSizePrefixed(header,
                                      ouster::OSF::osfHeaderIdentifier());
    std::ofstream myfile;
    myfile.open(filename,
                std::fstream::out | std::fstream::in | std::fstream::ate);
    if (myfile.is_open()) {
        myfile.seekp(0);
        myfile.write((char*)header_builder.GetBufferPointer(),
                     header_builder.GetSize());
        myfile.close();
    } else {
        std::cout << "fail to open " << filename << std::endl;
    }
}

size_t readPrefixedSizeFromOffset(const uint8_t* osf_file, size_t offset) {
    return osf_file[offset + 0] + (osf_file[offset + 1] << 8u) +
           (osf_file[offset + 2] << 16u) + (osf_file[offset + 3] << 24u);
}

std::string to_string(const uint8_t* buf, const size_t count) {
    std::stringstream ss;
    ss << std::hex;
    for (size_t i = 0; i < count; ++i) {
        if (i > 0) ss << " ";
        ss << std::setfill('0') << std::setw(2) << static_cast<int>(buf[i]);
    }
    return ss.str();
}

// ======= util_impl functions ========

template <typename T>
std::vector<T> vector_from_fb_vector(const flatbuffers::Vector<T>* fb_vec) {
    std::vector<T> res;

    // Field can be absent in Flatbuffer table
    if (fb_vec == nullptr) return res;

    res.reserve(fb_vec->size());
    for (auto el : *fb_vec) {
        res.emplace_back(el);
    }
    return res;
}

std::unique_ptr<sensor> sensor_from_osf_sensor(const osf_sensor* os) {
    std::unique_ptr<OSF::sensor> s(new OSF::sensor());
    s->id = os->id();

    // Fill sensor_info from fb osf_sensor
    auto s_info = os::default_sensor_info(lidar_mode_from_osf_mode(os->mode()));
    if (os->name()) s_info.name = os->name()->str();
    s_info.sn = (os->sn() != nullptr ? os->sn()->str() : std::string());
    s_info.fw_rev = (os->firmware_rev() != nullptr ? os->firmware_rev()->str()
                                                   : std::string());
    s_info.prod_line =
        (os->prod_line() != nullptr ? os->prod_line()->str() : std::string());
    if (os->lidar_origin_to_beam_origin_mm()) {
        s_info.lidar_origin_to_beam_origin_mm =
            os->lidar_origin_to_beam_origin_mm();
    }
    s_info.beam_azimuth_angles = vector_from_fb_vector(os->beam_azimuths());
    s_info.beam_altitude_angles = vector_from_fb_vector(os->beam_altitudes());

    const auto v_imu_to_sensor =
        vector_from_fb_vector(os->imu_to_sensor_transform());
    if (v_imu_to_sensor.size() == 16) {
        s_info.imu_to_sensor_transform =
            Eigen::Map<const os::mat4d>(v_imu_to_sensor.data()).transpose();
    }

    const auto v_lidar_to_sensor =
        vector_from_fb_vector(os->lidar_to_sensor_transform());
    if (v_lidar_to_sensor.size() == 16) {
        s_info.lidar_to_sensor_transform =
            Eigen::Map<const os::mat4d>(v_lidar_to_sensor.data()).transpose();
    }

    const auto v_extrinsics = vector_from_fb_vector(os->extrinsics());
    if (v_extrinsics.size() == 16) {
        s_info.extrinsic =
            Eigen::Map<const os::mat4d>(v_extrinsics.data()).transpose();
    }

    if (os->df_pixels_per_column()) {
        s_info.format.pixels_per_column = os->df_pixels_per_column();
    }

    if (os->df_columns_per_packet()) {
        s_info.format.columns_per_packet = os->df_columns_per_packet();
    }

    if (os->df_columns_per_frame()) {
        s_info.format.columns_per_frame = os->df_columns_per_frame();
    }

    const auto df_pixel_shift_by_row =
        vector_from_fb_vector(os->df_pixel_shift_by_row());
    if (!df_pixel_shift_by_row.empty()) {
        s_info.format.pixel_shift_by_row = df_pixel_shift_by_row;
    }

    s->meta = s_info;
    return s;
}

os::lidar_mode lidar_mode_from_osf_mode(OSF::LIDAR_MODE osf_mode) {
    // check boundaries and return default (because we don't have unknown mode)
    // in os::lidar_mode
    if (osf_mode <= LIDAR_MODE_MODE_UNKNOWN ||
        osf_mode > LIDAR_MODE_MODE_2048x10)
        return os::MODE_1024x10;
    return static_cast<os::lidar_mode>(osf_mode);
}

OSF::LIDAR_MODE osf_mode_from_lidar_mode(os::lidar_mode lidar_mode) {
    if (lidar_mode <= os::MODE_UNSPEC || lidar_mode > os::MODE_2048x10)
        return OSF::LIDAR_MODE_MODE_UNKNOWN;
    return static_cast<OSF::LIDAR_MODE>(lidar_mode);
}

OSF::OSF_FRAME_MODE osf_frame_mode_of_string(const std::string& frame_mode) {
    for (size_t i = 0; i <= OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_MAX; ++i) {
        if (OSF::EnumNamesOSF_FRAME_MODE()[i] == frame_mode)
            return OSF::EnumValuesOSF_FRAME_MODE()[i];
    }
    return OSF::OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_UNKNOWN;
}

std::string to_string(const OSF_FRAME_MODE frame_mode) {
    return OSF::EnumNamesOSF_FRAME_MODE()[frame_mode];
}

std::map<uint64_t, size_t> get_map_from_file_info(const FileInfo& file_info) {
    auto osf_session = get_osf_session_from_buf(file_info.buf());
    // Using map because ordered key are important (it is one of frame indexes
    // or timestamps)
    std::map<uint64_t, size_t> res;
    if (osf_session->map() == nullptr) return res;
    std::transform(osf_session->map()->begin(), osf_session->map()->end(),
                   std::inserter(res, res.begin()), [](const chunk_offset* el) {
                       return std::make_pair(el->index(), el->offset());
                   });
    return res;
}

/**
 * Fixing the destagerring to return correct lidar_scan column orders.
 * Can be applied only as last correction step of lidar scan recovery from
 * OSF V10 (usually warden's prod data).
 *
 * This fix needed for all OSF v10 (soft best guess criteria) because
 * the directions and amounts of px_offset changed with fw1.4 update.
 *
 * Production wardens keep's generating data in OSFs V10 that destaggered
 * to one direction and with current master code (as of Sept 20, 2020) during
 * the LidarScan recovery it's destaggered to the same direction with inverted
 * px_offset values. Which leads to all pixels in lidar scans be shifted to
 * <cycle count> pixels. (18 pixels for w == 1024)
 *
 * Where <cycle count> is the previous px_offset + inverted/reciprocals
 * px_offset for any given row. NOTE: by construction these values are the same
 * for every row so we can set the `fix_offset` uniformly to `px_offset`.
 *
 */
void fix_lidar_scan_destagerring_v10(LidarScan& ls) {
    int fix_offset;
    switch (ls.w) {
        case 512:
        default:
            fix_offset = 9;
            break;
        case 1024:
            fix_offset = 18;
            break;
        case 2048:
            fix_offset = 36;
            break;
    }
    // Uniform px_offset that is the cycle of px_offset + inverted(px_offset)
    std::vector<int> helper_px_offset(ls.h, fix_offset);
    // Shifting columns back to the full cycle
    ls.range() = destagger<LidarScan::raw_t, 1>(ls.range(), helper_px_offset);
    ls.noise() = destagger<LidarScan::raw_t, 1>(ls.noise(), helper_px_offset);
    ls.intensity() =
        destagger<LidarScan::raw_t, 1>(ls.intensity(), helper_px_offset);
    ls.reflectivity() =
        destagger<LidarScan::raw_t, 1>(ls.reflectivity(), helper_px_offset);
}

std::unique_ptr<LidarScan> lidar_scan_from_osf_message(
    const StampedMessage* msg, const FileInfo& file_info) {
    auto sensor = file_info.sensors().at(msg->id());

    uint32_t width = sensor->meta.format.columns_per_frame;
    uint32_t height = sensor->meta.format.pixels_per_column;

    // Init lidar scan
    std::unique_ptr<LidarScan> ls(new LidarScan(width, height));

    // Set timestamps per column
    auto msg_ts_vec = msg->message_as_lidar_scan()->ts();
    if (msg_ts_vec) {
        if (ls->ts.size() == msg_ts_vec->size()) {
            for (size_t i = 0; i < width; ++i) {
                ls->ts[i] = LidarScan::ts_t(msg_ts_vec->Get(i));
            }
        } else if (msg_ts_vec->size() != 0) {
            // NOTE[pb]: Zero size of ts vector can be present as the result of
            // frameChunker earlier implementation that recorded empty
            // ts vector.
            std::cout << "ERROR: lidar_scan msg has ts of length: "
                      << msg_ts_vec->size() << ", expected: " << ls->ts.size()
                      << std::endl;
            return nullptr;
        }
    }

    // Fill Scan Data with scan channels
    auto msg_scan_vec = msg->message_as_lidar_scan()->scan();
    if (!msg_scan_vec || !msg_scan_vec->size()) {
        std::cout
            << "ERROR: lidar_scan msg doesn't have scan field or it's empty.\n";
        return nullptr;
    }
    ScanData scan_data;
    for (size_t i = 0; i < msg_scan_vec->size(); ++i) {
        auto channel_buffer = msg_scan_vec->Get(i)->buffer();
        scan_data.emplace_back(channel_buffer->begin(), channel_buffer->end());
    }

    // Decode PNGs data to LidarScan
    if (scanDecode(*ls, scan_data, sensor->meta.format.pixel_shift_by_row,
                   file_info.lidar_frame_mode(),
                   file_info.range_multiplier())) {
        return nullptr;
    }

    // TODO[pb]: Later we might want to remove this in favor of one (or
    //           multiple) OSF conversion steps. But we don't have all needed
    //           tools for writing OSF convertors quickly now.
    // TODO[pb]: There also no ls.ts values in LidarScans so later we can add
    //           those to as part of convertors if they needed.
    // Fix LidarScan destagger for OSF V10 (e.g. prod wardens).
    if (file_info.version() == OSF_VERSION::V_1_0) {
        fix_lidar_scan_destagerring_v10(*ls);
    }

    return ls;
}

std::unique_ptr<Gps> gps_from_osf_message(const StampedMessage* msg) {
    auto gpsm = msg->message_as_gps_waypoint();
    return std::unique_ptr<Gps>(new Gps(
        gpsm->latitude(), gpsm->epy(), gpsm->longitude(), gpsm->epx(),
        gpsm->altitude(), gpsm->epv(), gpsm->ept(), gpsm->speed(), gpsm->eps(),
        gpsm->track(), gpsm->epd(), gpsm->climb(), gpsm->epc()));
}

std::unique_ptr<Imu> imu_from_osf_message(const StampedMessage* msg) {
    auto imum = msg->message_as_imu();
    std::unique_ptr<Imu> imu(new Imu());
    imu->angular_vel.push_back(imum->angular_vel()->x());
    imu->angular_vel.push_back(imum->angular_vel()->y());
    imu->angular_vel.push_back(imum->angular_vel()->z());
    imu->linear_accel.push_back(imum->linear_accel()->x());
    imu->linear_accel.push_back(imum->linear_accel()->y());
    imu->linear_accel.push_back(imum->linear_accel()->z());
    std::transform(imum->ts()->begin(), imum->ts()->end(),
                   std::back_inserter(imu->ts),
                   [](uint64_t t) { return std::chrono::nanoseconds(t); });
    return imu;
}

std::unique_ptr<Trajectory> traj_from_osf_message(const StampedMessage* msg) {
    auto traj = msg->message_as_trajectory();
    std::unique_ptr<Trajectory> res(new Trajectory());
    if (traj->poses() == nullptr) return res;
    std::transform(traj->poses()->begin(), traj->poses()->end(),
                   std::back_inserter(*res), [](const pose_f* osf_pose) {
                       return OSF::pose(osf_pose->R_w(), osf_pose->R_x(),
                                        osf_pose->R_y(), osf_pose->R_z(),
                                        osf_pose->T_x(), osf_pose->T_y(),
                                        osf_pose->T_z());
                   });
    return res;
}

std::pair<double, double> calc_traj_error(OSF::Trajectory& gt_traj,
                                          OSF::Trajectory& traj) {
    if (gt_traj.empty() || gt_traj.size() != traj.size()) {
        throw std::logic_error(
            "expect trajectories of equal size and non-empty");
    }
    double rot_err_acc = 0;
    double trans_err_acc = 0;
    const size_t W = traj.size();
    for (size_t v = 0; v < W; ++v) {
        auto gt_pose = gt_traj.at(v);
        auto pose = traj.at(v);

        double rot_error =
            1 - std::abs(gt_pose.orientation.dot(pose.orientation));
        double trans_error =
            (pose.position * gt_pose.position.inverse()).vector().norm();

        rot_err_acc += rot_error;
        trans_err_acc += trans_error;
    }
    return std::make_pair(rot_err_acc / W, trans_err_acc / W);
}

int get_max_lidar_frequency(const FileInfo::sensors_map& sensors) {
    int max_freq = 0;
    for (const auto& s : sensors) {
        max_freq =
            std::max(max_freq, frequency_of_lidar_mode(s.second->meta.mode));
    }
    return max_freq;
}

OSF::LIDAR_MODE osf_lidar_mode_of_string(const std::string& mode) {
    std::string mode_hack = "MODE_" + mode;
    for (size_t i = OSF::LIDAR_MODE_MIN; i <= OSF::LIDAR_MODE_MAX; ++i) {
        if (OSF::EnumNamesLIDAR_MODE()[i] == mode_hack)
            return OSF::EnumValuesLIDAR_MODE()[i];
    }
    return OSF::LIDAR_MODE_MODE_UNKNOWN;
}

OSF::ts_t get_max_frame_duration(const OSF::sensors_map& sensors) {
    const int max_freq = get_max_lidar_frequency(sensors);
    return OSF::ts_t(OSF::ts_t::period::den / max_freq);
}

flatbuffers::Offset<OSF::osf_sensor> create_osf_sensor(
    flatbuffers::FlatBufferBuilder& fbb, const OSF::sensor& sensor) {
    std::vector<double> imu_to_sensor(16);
    std::vector<double> lidar_to_sensor(16);
    std::vector<double> extrinsic(16);

    // Changing column-major data layout to row-major before storing to OSF buf
    // Not using internal mat4d::data() reprensentation here because we
    // don't always control how it is created.
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 4; j++) {
            imu_to_sensor[4 * i + j] =
                sensor.meta.imu_to_sensor_transform(i, j);
            lidar_to_sensor[4 * i + j] =
                sensor.meta.lidar_to_sensor_transform(i, j);
            extrinsic[4 * i + j] = sensor.meta.extrinsic(i, j);
        }
    }

    auto f_ext = fbb.CreateVector(extrinsic);
    auto f_alt = fbb.CreateVector(sensor.meta.beam_altitude_angles);
    auto f_azi = fbb.CreateVector(sensor.meta.beam_azimuth_angles);
    auto f_imu = fbb.CreateVector(imu_to_sensor);
    auto f_lid = fbb.CreateVector(lidar_to_sensor);
    auto name = fbb.CreateString(sensor.meta.name);
    auto hw_rev = fbb.CreateString(sensor.meta.fw_rev);
    auto sn = fbb.CreateString(sensor.meta.sn);

    auto prod_line = fbb.CreateString(sensor.meta.prod_line);

    auto lidar_origin_to_beam = sensor.meta.lidar_origin_to_beam_origin_mm;

    // pack data_format fields
    auto px_per_column = sensor.meta.format.pixels_per_column;
    auto cols_per_packet = sensor.meta.format.columns_per_packet;
    auto cols_per_frame = sensor.meta.format.columns_per_frame;
    auto f_px_off = fbb.CreateVector(sensor.meta.format.pixel_shift_by_row);

    return Createosf_sensor(fbb, name, sensor.id, sn, hw_rev,
                            osf_mode_from_lidar_mode(sensor.meta.mode), f_ext,
                            f_azi, f_alt, f_imu, f_lid, lidar_origin_to_beam,
                            prod_line, px_per_column, cols_per_packet,
                            cols_per_frame, f_px_off);
}

void build_session(flatbuffers::FlatBufferBuilder& fbb, const std::string& id,
                   const OSF::OSF_SESSION_MODE mode,
                   const OSF::OSF_FRAME_MODE lidar_frame_mode,
                   const uint8_t range_multiplier,
                   const OSF::sensors_map& sensors, const OSF::ts_t start_ts,
                   const OSF::ts_t end_ts,
                   const std::map<uint64_t, size_t>& id_to_offset) {
    std::vector<flatbuffers::Offset<ouster::OSF::osf_sensor>> osf_sensors;
    for (const auto& s : sensors) {
        if (s.first != s.second->id) {
            std::cout << "\nWARNING: Inconsistent OSF::sensors_map spotted "
                      << "during osfSession creation: map id = " << s.first
                      << " shoud be equal to sensor.id = " << s.second->id
                      << std::endl
                      << std::endl;
        }
        osf_sensors.push_back(create_osf_sensor(fbb, *s.second));
    }
    auto sensors_vec = fbb.CreateVector(osf_sensors);

    std::vector<ouster::OSF::chunk_offset> chunks;
    for (auto offset : id_to_offset) {
        chunks.emplace_back(offset.first, offset.second);
    }
    auto chunk_map = fbb.CreateVectorOfStructs(chunks);

    auto session_id = fbb.CreateString(id);

    auto session = CreateosfSession(
        fbb, session_id, mode, lidar_frame_mode, range_multiplier, sensors_vec,
        start_ts.count(), end_ts.count(), chunk_map);
    fbb.FinishSizePrefixed(session, ouster::OSF::osfSessionIdentifier());
}

std::ostream& operator<<(std::ostream& os, const LidarScanStat& lss) {
    using idx = LidarScan::index_t;

    os << "LidarScanStat [" << lss.w() << "x" << lss.h() << "], ts = ("
       << lss.ts_min().count() << ", " << lss.ts_max().count()
       << "), dt = " << lss.duration().count() << " :\n";

    // Buffer for snprintf
    const int kBufSize = 256;
    char sbuf[kBufSize];

    snprintf(sbuf, kBufSize, "%27s%15s%15s%15s\n", "RANGE", "INTENSITY",
             "NOISE", "REFLECTIVITY");
    os << sbuf;

    snprintf(sbuf, kBufSize, "%12s", "mean");
    os << sbuf;
    for (idx i = 0; i < static_cast<idx>(lss.mean().size()); ++i) {
        snprintf(sbuf, kBufSize, "% 15.3f", lss.mean()(i));
        os << sbuf;
    }

    os << std::endl;

    snprintf(sbuf, kBufSize, "%12s", "min");
    os << sbuf;
    for (idx i = 0; i < static_cast<idx>(lss.min().size()); ++i) {
        snprintf(sbuf, kBufSize, "% 15.3f", lss.min()(i));
        os << sbuf;
    }

    os << std::endl;

    snprintf(sbuf, kBufSize, "%12s", "max");
    os << sbuf;
    for (idx i = 0; i < static_cast<idx>(lss.max().size()); ++i) {
        snprintf(sbuf, kBufSize, "% 15.3f", lss.max()(i));
        os << sbuf;
    }

    os << std::endl << std::endl;

    snprintf(sbuf, kBufSize, "%12s", "cov");
    os << sbuf;
    const LidarScanStat::cov_t& cov = lss.covMatrix();
    for (idx r = 0; r < 4; ++r) {
        if (r > 0) {
            snprintf(sbuf, kBufSize, "\n%12s", "");
            os << sbuf;
        }

        for (idx c = 0; c < 4; ++c) {
            snprintf(sbuf, kBufSize, "% 15.3f", cov(r, c));
            os << sbuf;
        }
    }

    return os << std::endl;
}

std::ostream& operator<<(std::ostream& os, const LidarScan& ls) {
    return os << LidarScanStat{ls};
}

}  // namespace OSF
}  // namespace ouster
