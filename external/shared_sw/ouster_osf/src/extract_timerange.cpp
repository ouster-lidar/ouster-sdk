#include <flatbuffers/util.h>
#include <json/json.h>
#include <yaml-cpp/yaml.h>

#include <iostream>

#include "chunk.h"
#include "osfChunk_generated.h"
#include "osfHeader_generated.h"
#include "osfSession_generated.h"
#include "ouster/osf/util.h"
#include "ouster/osf/version.h"
#include "util_impl.h"

std::vector<std::string> parseMetaOsf(std::string file, size_t start,
                                      size_t stop) {
    std::vector<std::string> ret;

    try {
        YAML::Node meta = YAML::LoadFile(file);
        std::multimap<size_t, std::string> map;

        for (auto a : meta) {
            auto name = a.first.as<std::string>();
            std::cerr << name << std::endl;
            auto file_start = meta[name]["start_ts"].as<size_t>();
            auto file_stop = meta[name]["stop_ts"].as<size_t>();
            if (!((file_start < start && file_stop < start) ||
                  (file_start > stop && file_stop > stop))) {
                map.emplace(file_start, name);
            }
        }
        for (auto name : map) ret.emplace_back(name.second);
        return ret;
    } catch (const std::exception& e) {
        std::cerr << "Error opening file : " << file << " : " << e.what()
                  << std::endl;
        return ret;
    }
}

void extractBuffer(const std::string& dest_file,
                   const std::vector<std::string>& file_paths,
                   const uint64_t& start, const uint64_t& stop) {
    if (file_paths.empty()) {
        std::cerr << "Empty IN_FILES. Please provide at least one osf as input."
                  << std::endl;
        return;
    }
    size_t header_size = ouster::OSF::initEmptyOsfFile(dest_file);

    std::map<uint64_t, size_t> ts_to_offset;
    size_t current_chunks_offset = 0;

    const ouster::OSF::osfSession* my_osfSession = nullptr;
    uint64_t extract_start = 0xffffffffffffffff;
    uint64_t extract_stop = 0;

    for (auto file_path : file_paths) {  // Parsing all files
        auto opener = ouster::OSF::openOsfBuffer(file_path.c_str());
        uint8_t* osf_file = opener.osf_file;

        my_osfSession = ouster::OSF::GetSizePrefixedosfSession(
            osf_file + opener.session_offset);

        if (my_osfSession->session_mode() ==
            ouster::OSF::OSF_SESSION_MODE_OSF_STREAM) {
            // create chunk
            auto chunk = ouster::OSF::OSFChunk(
                5 * 1024 * 1024, my_osfSession->lidar_frame_mode(),
                my_osfSession->range_multiplier());
            bool new_chunk = true;

            uint64_t current_offset = opener.chunks_offset;
            while (current_offset < opener.session_offset) {
                auto my_osf = ouster::OSF::GetSizePrefixedosfChunk(
                    osf_file + current_offset);
                // Get the size of the buffer
                uint32_t prefixed_size =
                    ouster::OSF::readPrefixedSizeFromOffset(osf_file,
                                                            current_offset);
                for (auto frame : *my_osf->frames()) {
                    auto ts = frame->ts();
                    if (ts >= start && ts <= stop) {
                        if (new_chunk) {
                            ts_to_offset[ts] = current_chunks_offset;
                            new_chunk = false;
                        }
                        chunk.logMessage(frame);
                        if (ts < extract_start) extract_start = ts;
                        if (ts > extract_stop) extract_stop = ts;
                        if (chunk.checkChunkSize(ts)) {
                            current_chunks_offset += chunk.saveChunk(dest_file);
                            new_chunk = true;
                        }
                    }
                }
                current_offset +=
                    prefixed_size + ouster::OSF::SIZE_OF_PREFIXED_SIZE;
            }

            // Save the last chunk
            if (!new_chunk) {
                current_chunks_offset += chunk.saveChunk(dest_file);
            }

        } else if (my_osfSession->session_mode() ==
                   ouster::OSF::OSF_SESSION_MODE_OSF_FRAMED) {
            uint64_t current_offset = opener.chunks_offset;
            while (current_offset < opener.session_offset) {
                auto my_osf = ouster::OSF::GetSizePrefixedosfChunk(
                    osf_file + current_offset);
                // Get the size of the buffer
                uint32_t prefixed_size =
                    ouster::OSF::readPrefixedSizeFromOffset(osf_file,
                                                            current_offset);

                if (!((my_osf->start_ts() < start &&
                       my_osf->end_ts() < start) ||
                      (my_osf->start_ts() > stop && my_osf->end_ts() > stop))) {
                    ouster::OSF::savePrefixedSizeFlatBuffer(
                        (char*)flatbuffers::GetBufferStartFromRootPointer(
                            my_osf),
                        prefixed_size, dest_file, true);
                    ts_to_offset[my_osf->start_ts()] = current_chunks_offset;
                    if (my_osf->start_ts() < extract_start)
                        extract_start = my_osf->start_ts();
                    if (my_osf->end_ts() > extract_stop)
                        extract_stop = my_osf->end_ts();
                    current_chunks_offset +=
                        prefixed_size + ouster::OSF::SIZE_OF_PREFIXED_SIZE;
                }
                current_offset +=
                    prefixed_size + ouster::OSF::SIZE_OF_PREFIXED_SIZE;
                // if (my_osf->end_ts() < start) //early stop
                // break;
            }
        }
    }
    // create session header
    auto session_builder = flatbuffers::FlatBufferBuilder(1024);
    std::vector<flatbuffers::Offset<ouster::OSF::osf_sensor>> my_sensors;

    for (auto sensor : *my_osfSession->sensors()) {
        auto f_ext = session_builder.CreateVector(sensor->extrinsics()->data(),
                                                  sensor->extrinsics()->size());
        auto f_alt = session_builder.CreateVector(
            sensor->beam_altitudes()->data(), sensor->beam_altitudes()->size());
        auto f_azi = session_builder.CreateVector(
            sensor->beam_azimuths()->data(), sensor->beam_azimuths()->size());
        auto f_imu = session_builder.CreateVector(
            sensor->imu_to_sensor_transform()->data(),
            sensor->imu_to_sensor_transform()->size());
        auto f_lid = session_builder.CreateVector(
            sensor->lidar_to_sensor_transform()->data(),
            sensor->lidar_to_sensor_transform()->size());

        auto name = session_builder.CreateString(sensor->name()->str());
        auto hw_rev =
            session_builder.CreateString(sensor->firmware_rev()->str());
        auto sn = session_builder.CreateString(sensor->sn()->str());

        auto prod_line = session_builder.CreateString(sensor->prod_line());

        auto lidar_origin_to_beam = sensor->lidar_origin_to_beam_origin_mm();

        // pack data_format fields
        auto px_per_column = sensor->df_pixels_per_column();
        auto cols_per_packet = sensor->df_columns_per_packet();
        auto cols_per_frame = sensor->df_columns_per_frame();

        flatbuffers::Offset<flatbuffers::Vector<int32_t>> f_px_off(0);
        if (sensor->df_pixel_shift_by_row()) {
            f_px_off = session_builder.CreateVector(
            sensor->df_pixel_shift_by_row()->data(),
            sensor->df_pixel_shift_by_row()->size());
        }

        my_sensors.push_back(ouster::OSF::Createosf_sensor(
            session_builder, name, sensor->id(), sn, hw_rev, sensor->mode(),
            f_ext, f_azi, f_alt, f_imu, f_lid, lidar_origin_to_beam, prod_line,
            px_per_column, cols_per_packet, cols_per_frame, f_px_off));
    }

    auto sensors = session_builder.CreateVector(my_sensors);

    std::vector<ouster::OSF::chunk_offset> chunks;
    for (auto offset : ts_to_offset) {
        chunks.emplace_back(offset.first, offset.second);
    }
    auto chunk_map = session_builder.CreateVectorOfStructs(chunks);

    auto id = session_builder.CreateString(my_osfSession->id()->str());

    if (extract_start == 0xffffffffffffffff) extract_start = 0;

    auto session = CreateosfSession(
        session_builder, id, ouster::OSF::OSF_SESSION_MODE_OSF_STREAM,
        my_osfSession->lidar_frame_mode(), my_osfSession->range_multiplier(),
        sensors, extract_start, extract_stop, chunk_map);
    session_builder.FinishSizePrefixed(session,
                                       ouster::OSF::osfSessionIdentifier());
    auto session_size = session_builder.GetSize();
    ouster::OSF::saveFlatBuffer(session_builder, dest_file, true);
    ouster::OSF::closeOsfFile(dest_file, current_chunks_offset + header_size,
                              session_size);
    chunks.clear();
    my_sensors.clear();
}

static void show_usage(const std::string& name) {
    std::cerr << "Usage: " << name
              << " START_TS STOP_TS DEST_FILE [IN_FILES…]\n"
              << "START_TS           : begining of the extraction \n"
              << "STOP_TS            : end of the extraction\n"
              << "DEST_FILE          : file to save the extracted data\n"
              << "IN_FILES           : osf files to parse\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 5) {
        show_usage(argv[0]);
        return 0;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_usage(argv[0]);
            return 0;
        }
    }

    size_t start_ts = std::stoull(argv[1]);
    size_t stop_ts = std::stoull(argv[2]);
    std::string dest_file = argv[3];
    std::vector<std::string> files;
    for (int i = 4; i < argc; i++) {
        files.emplace_back(argv[i]);
    }

    extractBuffer(dest_file, files, start_ts, stop_ts);
    return 0;
}
