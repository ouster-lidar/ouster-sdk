/**
 * @file
 *
 * Higher-level operations on OSF files. TODO: Fix interfaces. These were
 * directly extracted from CLI tools for reuse without modification.
 */

#include <json/json.h>

#include <fstream>
#include <iomanip>
#include <memory>

#include "flatbuffers/util.h"
#include "frame.h"
#include "osfChunk_generated.h"
#include "osfSession_generated.h"
#include "ouster/osf/file_info.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/util.h"
#include "ouster/osf/version.h"
#include "util_impl.h"

namespace ouster {
namespace OSF {

static constexpr int FRAME_ID_DIGITS_ = 6;

// write legacy OSF metadata json
static Json::Value osfToLegacyMetadataJson(
    const ouster::OSF::osf_sensor* sensor, int range_multiplier,
    int frame_frequency = 1) {
    Json::Value root{};

    root["version"] = ouster::OSF::OSF_VERSION::V_1_1;
    root["prod_sn"] = sensor->sn();
    root["range_scale"] = range_multiplier;
    root["build_rev"] = sensor->firmware_rev()->str();
    root["frame_frequency"] = frame_frequency;

    for (auto a : *sensor->beam_altitudes())
        root["elevation_angles_rad"].append(a * M_PI / 180.0);
    for (auto a : *sensor->beam_azimuths())
        root["azimuth_angles_rad"].append(a * M_PI / 180.0);
    for (const double a : *sensor->extrinsics()) root["extrinsics"].append(a);

    return root;
}

static void writeJson(const std::string& filename, const Json::Value& root) {
    Json::StreamWriterBuilder builder;
    builder["enableYAMLCompatibility"] = true;
    builder["precision"] = 8;
    builder["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer{builder.newStreamWriter()};

    std::ofstream jout(filename);
    writer->write(root, &jout);
    jout << std::endl;
}

static std::string getFileName(const std::string& sensor_name,
                               const int& frame_id, const std::string& extn,
                               const std::string& channel = "") {
    std::string channel_separator = "";
    if (!channel.empty()) channel_separator = "_";
    std::stringstream filename;
    filename << "/" + channel + channel_separator + sensor_name << "_"
             << std::setfill('0') << std::setw(FRAME_ID_DIGITS_) << frame_id
             << "." << extn;
    return filename.str();
}

void stream2LegacyOsf(const std::string& file, const std::string& dest) {
    auto opener = ouster::OSF::openOsfBuffer(file.c_str());
    uint8_t* osf_file = opener.osf_file;
    std::cout << "Converting fb to Png and json" << std::endl;

    auto my_osfSession = ouster::OSF::GetSizePrefixedosfSession(
        osf_file + opener.session_offset);
    // create metadata

    for (auto sensor : *my_osfSession->sensors()) {
        auto root = osfToLegacyMetadataJson(
            sensor, ouster::OSF::RANGE_MULTIPLIER_DEFAULT);
        writeJson(dest + "/" + sensor->name()->str() + ".json", root);
    }

    auto frame_mode = my_osfSession->lidar_frame_mode();

    uint64_t current_offset = opener.chunks_offset;

    int frameId = 0;
    while (current_offset < opener.session_offset) {
        auto my_osf =
            ouster::OSF::GetSizePrefixedosfChunk(osf_file + current_offset);
        std::map<uint64_t, int> tsCount;

        for (auto frame : *my_osf->frames()) {
            if (frame->message_type() == ouster::OSF::Message_lidar_scan) {
                switch (frame_mode) {
                    case ouster::OSF::OSF_FRAME_MODE_OSF_32: {
                        std::fstream myfile;
                        std::string filename =
                            dest + getFileName(my_osfSession->sensors()
                                                   ->Get(frame->id() - 1)
                                                   ->name()
                                                   ->str(),
                                               frameId, "png");
                        myfile.open(filename, std::fstream::out |
                                                  std::fstream::trunc |
                                                  std::fstream::binary);
                        if (myfile.is_open()) {
                            myfile.write((char*)frame->message_as_lidar_scan()
                                             ->scan()
                                             ->Get(0)
                                             ->buffer()
                                             ->data(),
                                         frame->message_as_lidar_scan()
                                             ->scan()
                                             ->Get(0)
                                             ->buffer()
                                             ->size());
                            myfile.close();
                        } else {
                            std::cout << "fail to open : " << filename
                                      << std::endl;
                        }
                        break;
                    }

                    default:
                        std::cerr
                            << "OSF_FRAME_MODE not supported in current version"
                            << std::endl;
                        break;
                }
            } else if (frame->message_type() ==
                       ouster::OSF::Message_trajectory) {
                Json::Value root{};
                Json::Value trajectory{};
                for (auto p : *frame->message_as_trajectory()->poses()) {
                    trajectory.clear();
                    trajectory.append(p->R_w());
                    trajectory.append(p->R_x());
                    trajectory.append(p->R_y());
                    trajectory.append(p->R_z());
                    trajectory.append(p->T_x());
                    trajectory.append(p->T_y());
                    trajectory.append(p->T_z());
                    root["trajectory"].append(trajectory);
                }

                root["ts"] = std::to_string(
                    frame->ts());  // as string to avoid the loss of
                                   // precision applied to the JSON

                Json::StreamWriterBuilder builder;
                builder["enableYAMLCompatibility"] = true;
                builder["precision"] = 8;
                builder["indentation"] = "  ";
                std::unique_ptr<Json::StreamWriter> writer{
                    builder.newStreamWriter()};

                std::ofstream jout(dest + getFileName(my_osfSession->sensors()
                                                          ->Get(frame->id() - 1)
                                                          ->name()
                                                          ->str(),
                                                      frameId, "json"));
                writer->write(root, &jout);
                jout.close();
            }
        }

        // Get the size of the buffer
        uint32_t prefixed_size =
            ouster::OSF::readPrefixedSizeFromOffset(osf_file, current_offset);
        current_offset += prefixed_size + ouster::OSF::SIZE_OF_PREFIXED_SIZE;
        frameId += 1;
    }
}

void rechunk(std::string file, std::string dest_file,
             const OSF::OSF_FRAME_MODE output_frame_mode,
             const bool use_car_trajectory) {
    OsfFile osf_file(file);

    if (!osf_file.good()) {
        return;
    }

    Reader reader(osf_file);

    OSF::OSF_FRAME_MODE frame_mode = output_frame_mode;

    if (frame_mode == OSF_FRAME_MODE::OSF_FRAME_MODE_OSF_UNKNOWN) {
        frame_mode = reader.file_info().lidar_frame_mode();
    }

    if (frame_mode != reader.file_info().lidar_frame_mode()) {
        std::cout << "Recoding lidar frame mode: "
                  << to_string(reader.file_info().lidar_frame_mode()) << " -> "
                  << to_string(frame_mode) << std::endl;
    }

    // Stream cutter that defines the start of every frame
    SortedWindow ls_queue(reader, SORT_WINDOW_SIZE_LIDAR_SCAN,
                          {OSF::MessageType::LIDAR_SCAN});

    TrajectoryReader car_traj_reader(reader);

    // All other messages organized in a sorted queues of various window sizes
    // based on their frequency characteristics
    std::vector<std::unique_ptr<SortedWindow>> other_queues;

    // Trajectory message currently goes together with lidar_scan + car
    // trajectory so double the size of LIDAR_SCAN window should be enough
    other_queues.emplace_back(new SortedWindow(
        reader, SORT_WINDOW_SIZE_TAJECTORY,
        {OSF::MessageType::TRAJECTORY, OSF::MessageType::MESSAGE_EXTENSION}));

    // Imu messages recorded with 10 times the frequency of lidar_scan
    // so we need to have a bigger window size, empirically x5 works well
    // but in the case of appearing skipped() messages in the final output
    // WARNING we migh increase this number
    other_queues.emplace_back(new SortedWindow(reader, SORT_WINDOW_SIZE_IMU,
                                               {OSF::MessageType::IMU}));

    // Gps is the most infrequent message with 1 Hz so the small window size
    other_queues.emplace_back(new SortedWindow(
        reader, SORT_WINDOW_SIZE_GPS, {OSF::MessageType::GPS_WAYPOINT}));

    // FrameSaver operates on sorted messages sequences with LIDAR_SCAN messages
    // as a frame cutters which should alway goes first until Status::FINISH
    // reached
    FramesSaver frames_saver(dest_file, reader.file_info().sensors(),
                             reader.file_info().id(), frame_mode,
                             reader.file_info().range_multiplier());

    // Convenience method to add all current messages from all queues
    auto log_other_msgs = [&frames_saver, &other_queues]() {
        for (const auto& sw : other_queues) {
            while (!sw->empty() && (frames_saver.logMessage(sw->peek()) !=
                                    FramesSaver::Status::FINISH)) {
                sw->pop();
            }
        }
    };

    // Convenience method to add all messages from all queues without checking
    // for frame bounds (used only on the last frame to save outstanding msgs)
    auto save_all_other_msgs = [&frames_saver, &other_queues]() {
        for (const auto& sw : other_queues) {
            while (!sw->empty()) {
                frames_saver.saveMessage(sw->pop());
            }
        }
    };

    int generated_traj_cnt = 0;

    auto sensors = reader.file_info().sensors();

    // Use lidar_scans as the cutter frame knife
    while (!ls_queue.empty()) {
        const OSF::MessageRef m = ls_queue.peek();

        // Add lidar scan
        const auto status = frames_saver.logMessage(m);

        // Generate sensor trajectory from car trajectory if needed
        if (status == FramesSaver::Status::OK &&
            m.type() == OSF::MessageType::LIDAR_SCAN && use_car_trajectory) {
            auto ls = m.as_lidar_scan();
            if (ls) {
                auto ls_traj = car_traj_reader.evaluate(
                    ls->ts, sensors.at(m.id())->meta.extrinsic);
                if (ls_traj) {
                    // FramesSaver checks that we have only one traj per lidar
                    // scan with the same timestamp. (first in first to stay)

                    // Thus by adding generated sensor trjectory earlier we
                    // will bounce off any subsequent trajectories for the
                    // given sensor id and timestamp if they present in input
                    // OSF file.
                    if (frames_saver.logTrajectory(m.id(), *ls_traj, m.ts()) ==
                        FramesSaver::Status::OK) {
                        ++generated_traj_cnt;
                    }
                }
            }
        }

        if (status == FramesSaver::Status::FINISH) {
            // Reach the out of bounds for the current frame, need to finish it
            log_other_msgs();
            frames_saver.saveFrame();
            // ... repeat the loop with the same ls message in the queue
        } else {
            // OK or SKIPPED: move on to the next ls msg
            ls_queue.pop();
        }
    }

    // Add outstanding messages to the last frame
    save_all_other_msgs();

    // Create session object and finish OSF file with the correct header
    frames_saver.finish();

    // Warninng outputs (if any, check window sizes)
    if (ls_queue.skipped()) {
        std::cout << "WARNING: ls.skipped = " << ls_queue.skipped()
                  << std::endl;
    }

    // Check and show skipped numbers for all queues
    for (const auto& sw : other_queues) {
        if (sw->skipped()) {
            std::cout << "WARNING: Queue: skipped = ";
            std::cout << sw->skipped() << ", [ ";
            std::stringstream f_ss;
            for (auto f : sw->msg_filters()) {
                if (f_ss.rdbuf()->in_avail()) f_ss << ", ";
                f_ss << to_string(f);
            }
            std::cout << f_ss.str() << " ]" << std::endl;
        }
    }

    if (use_car_trajectory) {
        std::cout << "INFO: generated trajectories count = "
                  << generated_traj_cnt << std::endl;
    }

    if (frames_saver.skipped()) {
        std::cout << "INFO: frames_saver.skipped = " << frames_saver.skipped()
                  << std::endl;
    }
}

// Get framed OSF file statitstics.
FileStatsFramed calc_framed_file_stats(const std::string& file) {
    OsfFile osf_file_chunked(file);

    if (!osf_file_chunked.good()) {
        return FileStatsFramed();
    }

    Reader reader(osf_file_chunked);

    std::vector<uint64_t> f_keys = reader.file_info().frames_keys();
    FileInfo::sensors_map sensors = reader.file_info().sensors();

    FrameLidarScanTrajectoryCounter fm(sensors);

    // mean of dt between subsequent frames
    double m_s = 0;
    // variance of dt
    double m2_s = 0;
    // number of elements tracked for mean/variance calculations
    uint64_t cnt = 0;

    // Fields for FileStatsFramed
    uint64_t past_cnt = 0;
    uint64_t jumps_cnt = 0;
    uint64_t ls_total_cnt = 0;
    uint64_t ls_with_traj_cnt = 0;
    uint64_t full_frames_cnt = 0;
    uint64_t start_end_mismatch_cnt = 0;
    uint64_t overlapped_frames_cnt = 0;

    // Track session start/end that it should be equal to one in file_info
    FileInfo::ts_t session_start_ts(0);
    FileInfo::ts_t session_end_ts(0);

    FileInfo::ts_t prev_start_ts(0);
    FileInfo::ts_t prev_end_ts(0);
    for (uint64_t frame_index : f_keys) {
        const FileInfo::ts_t frame_start_ts = reader.start_ts(frame_index);
        const FileInfo::ts_t frame_end_ts = reader.end_ts(frame_index);

        // Collect cross frame stats
        if (prev_start_ts.count()) {
            double dt_s =
                (frame_start_ts.count() - prev_start_ts.count()) / 1e+9;
            if (dt_s > 24 * 3600) {
                // Time sync
                jumps_cnt++;
                // Reset running stats params
                cnt = 0;
                m_s = 0;
                m2_s = 0;
            } else {
                // Update running stats
                if (dt_s < 0) past_cnt++;
                cnt++;
                double m_s_prev = m_s;
                m_s += (dt_s - m_s) / cnt;
                // Welford's method for variance:
                // https://www.johndcook.com/blog/standard_deviation/
                m2_s += (dt_s - m_s_prev) * (dt_s - m_s);
            }

            overlapped_frames_cnt += (prev_end_ts >= frame_start_ts);
        }
        prev_start_ts = frame_start_ts;
        prev_end_ts = frame_end_ts;

        // Collect in-frame stats
        fm.reset();
        Reader::MessagesRange msgs = reader.messages(frame_index);
        FileInfo::ts_t end_ts(0);
        FileInfo::ts_t start_ts(0);
        for (const MessageRef& m : msgs) {
            if (m.type() == MessageType::LIDAR_SCAN) {
                ls_total_cnt++;
                fm.add_lidar_scan(m.id(), m.ts());
            }
            // Filter out non-sensor traj (e.g. >= 127)
            if (m.type() == MessageType::TRAJECTORY && m.id() < 127) {
                fm.add_trajectory(m.id(), m.ts());
            }
            // Update start/end_ts for the frame
            if (start_ts.count() == 0 || start_ts > m.ts()) start_ts = m.ts();
            if (end_ts < m.ts()) end_ts = m.ts();

            // Session start_ts
            if (session_start_ts.count() == 0 || session_start_ts > m.ts()) {
                session_start_ts = m.ts();
            }
            // Session end_ts
            if (session_end_ts < m.ts()) {
                session_end_ts = m.ts();
            }
        }
        start_end_mismatch_cnt +=
            (start_ts != frame_start_ts || end_ts != frame_end_ts);
        const uint64_t lswtc = fm.count_pairs();
        ls_with_traj_cnt += lswtc;
        full_frames_cnt += (lswtc == sensors.size());
    }

    FileStatsFramed fstats;
    fstats.dt_mean = m_s;
    fstats.dt_past_cnt = past_cnt;
    fstats.dt_jumps_cnt = jumps_cnt;
    fstats.ls_total_cnt = ls_total_cnt;
    fstats.ls_with_traj_cnt = ls_with_traj_cnt;
    fstats.full_frames_cnt = full_frames_cnt;
    fstats.frames_cnt = f_keys.size();
    fstats.start_end_mismatch_cnt = start_end_mismatch_cnt;
    fstats.overlapped_frames_cnt = overlapped_frames_cnt;

    fstats.session_start_ts = session_start_ts;
    fstats.session_end_ts = session_end_ts;

    fstats.full_frames_ratio =
        static_cast<double>(fstats.full_frames_cnt) / fstats.frames_cnt;

    if (cnt > 1) {
        fstats.dt_var = m2_s / (cnt - 1);
    } else {
        fstats.dt_var = 0;
    }

    return fstats;
}

std::string to_string(const FileStatsFramed& fstats) {
    std::stringstream ss;
    ss << "FileStatsFramed [ dt_mean = " << fstats.dt_mean << ", "
       << "dt_var = " << fstats.dt_var << ", "
       << "dt_past_cnt = " << fstats.dt_past_cnt << ", "
       << "dt_jumps_cnt = " << fstats.dt_jumps_cnt << ", "
       << "ls_total_cnt = " << fstats.ls_total_cnt << ", "
       << "ls_with_traj_cnt = " << fstats.ls_with_traj_cnt << ", "
       << "full_frames_cnt = " << fstats.full_frames_cnt << ", "
       << "start_end_mismatch_cnt = " << fstats.start_end_mismatch_cnt << ", "
       << "overlapped_frames_cnt = " << fstats.overlapped_frames_cnt << ", "
       << "session_start_ts = " << fstats.session_start_ts.count() << ", "
       << "session_end_ts = " << fstats.session_end_ts.count() << ", "
       << "full_frames_ratio = " << fstats.full_frames_ratio << ", "
       << "frames_cnt = " << fstats.frames_cnt << "]";
    return ss.str();
}

}  // namespace OSF
}  // namespace ouster
