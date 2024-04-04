/**
 * Copyright(c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/writerv2.h"

#include <stdexcept>

namespace ouster {
namespace osf {
WriterV2::WriterV2(const std::string& filename,
                   const ouster::sensor::sensor_info& info, uint32_t chunk_size,
                   const LidarScanFieldTypes& field_types)
    : WriterV2(filename, std::vector<ouster::sensor::sensor_info>{info},
               chunk_size, field_types) {}

WriterV2::WriterV2(const std::string& filename,
                   const std::vector<ouster::sensor::sensor_info>& info,
                   uint32_t chunk_size, const LidarScanFieldTypes& field_types)
    : filename(filename),
      info(info),
      chunk_size(chunk_size),
      field_types(field_types) {
    writer =
        std::make_unique<Writer>(filename, "New Writer Interface", chunk_size);

    for (uint32_t i = 0; i < sensor_info_count(); i++) {
        meta_id[i] = writer->addMetadata(LidarSensor(info[i]));
    }
}

void WriterV2::_save(uint32_t stream_index, const LidarScan& scan) {
    if (stream_index < sensor_info_count()) {
        auto item = streams.find(stream_index);
        if (item == streams.end()) {
            const auto& fields =
                field_types.size() ? field_types : get_field_types(scan);
            streams[stream_index] = std::make_unique<LidarScanStream>(
                *writer, meta_id[stream_index], fields);
        }
        ts_t time = ts_t(scan.get_first_valid_packet_timestamp());

        streams[stream_index]->save(time, scan);
    } else {
        throw std::logic_error("ERROR: Bad Stream ID");
    }
}

void WriterV2::save(uint32_t stream_index, const LidarScan& scan) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    _save(stream_index, scan);
}

void WriterV2::save(const std::vector<LidarScan>& scans) {
    if (is_closed()) {
        throw std::logic_error("ERROR: Writer is closed");
    }
    if (scans.size() != sensor_info_count()) {
        throw std::logic_error(
            "ERROR: Scans passed in to writer "
            "does not match number of sensor infos");
    } else {
        for (uint32_t i = 0; i < scans.size(); i++) {
            _save(i, scans[i]);
        }
    }
}

const std::vector<ouster::sensor::sensor_info>& WriterV2::get_sensor_info()
    const {
    return info;
}

const ouster::sensor::sensor_info WriterV2::get_sensor_info(
    int stream_index) const {
    return info[stream_index];
}

uint32_t WriterV2::sensor_info_count() const { return info.size(); }

const std::string& WriterV2::get_filename() const { return filename; }

uint32_t WriterV2::get_chunk_size() const { return chunk_size; }

void WriterV2::close() { writer.reset(nullptr); }

bool WriterV2::is_closed() const { return writer == nullptr; }

}  // namespace osf
}  // namespace ouster
