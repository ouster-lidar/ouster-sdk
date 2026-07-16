/**
 * Copyright(c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include "common.h"
#include "osf_test.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/osf/impl/compat_ops.h"
#include "ouster/osf/impl/fb_utils.h"
#include "ouster/osf/meta_streaming_info.h"
#include "ouster/osf/osf_frame_set_source.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/stream_lidar_frame.h"
#include "ouster/osf/stream_osf_file.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace sdk {
namespace osf {
namespace {

using ouster::sdk::core::SensorInfo;
using ouster::sdk::osf::get_random_lidar_frame;

class ReaderTimestampIdxTest : public OsfTestWithDataAndFiles {};

std::string write_minimal_streaming_osf(const std::string& test_data_dir,
                                        const std::string& output_path) {
    const SensorInfo sinfo = ouster::sdk::core::metadata_from_json(
        path_concat(test_data_dir, "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    Writer writer(output_path);
    const auto sensor_id = writer.add_sensor(sinfo);
    writer.save(sensor_id, get_random_lidar_frame(sinfo), ts_t{1});
    writer.save(sensor_id, get_random_lidar_frame(sinfo), ts_t{2});
    writer.close();
    return output_path;
}

void rewrite_streaming_info(const std::string& path, StreamingInfo streaming_info) {
    uint64_t metadata_offset = 0;
    ouster::sdk::core::Version version = ouster::sdk::osf::OsfFile::CURRENT_VERSION;
    flatbuffers::FlatBufferBuilder metadata_fbb(32768);
    {
        StreamOsfFile osf_file{path};
        version = osf_file.version();
        metadata_offset = osf_file.metadata_offset().offset();
    }

    // Scope the reader so the file handle is released before truncate on
    // Windows (_SH_DENYRW rejects concurrent opens).
    {
        Reader reader(path);
        MetadataStore new_store;
        for (const auto& item : reader.meta_store().entries()) {
            if (item.second->type() == metadata_type<StreamingInfo>()) {
                continue;
            }
            new_store.add(*item.second);
        }
        new_store.add(std::move(streaming_info));

        auto entries = impl::make_entries(new_store, metadata_fbb);

        std::vector<impl::gen::ChunkOffset> chunks;
        for (const auto& entry : reader.chunks()) {
            chunks.emplace_back(entry.start_ts().count(), entry.end_ts().count(), entry.offset());
        }

        const auto metadata = impl::gen::CreateMetadataDirect(
            metadata_fbb, reader.metadata_id().c_str(), reader.start_ts().count(),
            reader.end_ts().count(), &chunks, &entries);
        metadata_fbb.FinishSizePrefixed(metadata, impl::gen::MetadataIdentifier());
    }

    truncate_file(path, metadata_offset);
    const auto saved_bytes = static_cast<uint32_t>(impl::builder_to_file(metadata_fbb, path, true));
    impl::finish_osf_file(path, metadata_offset, saved_bytes, version);
}

TEST_F(ReaderTimestampIdxTest, FalseWhenStreamTimestampsIncomplete) {
    const std::string path =
        write_minimal_streaming_osf(test_data_dir(), tmp_file("ts_idx_bad.osf"));

    StreamingInfo corrupted;
    {
        Reader reader(path);
        auto streaming_info = reader.meta_store().get<StreamingInfo>();
        ASSERT_NE(streaming_info, nullptr);
        const auto lidar_stream_id = reader.meta_store().get<LidarFrameStreamMeta>()->id();

        corrupted = StreamingInfo(streaming_info->chunks_info(), streaming_info->stream_stats());
        auto& stats = corrupted.stream_stats().at(lidar_stream_id);
        ASSERT_EQ(stats.message_count, stats.receive_timestamps.size());
        stats.receive_timestamps.resize(1);
        ASSERT_LT(stats.receive_timestamps.size(), stats.message_count);
    }

    rewrite_streaming_info(path, corrupted);

    Reader corrupted_reader(path);
    EXPECT_TRUE(corrupted_reader.has_message_idx());
    EXPECT_FALSE(corrupted_reader.has_timestamp_idx());

    OsfFrameSetSource source(path);
    EXPECT_FALSE(source.is_indexed());
}

TEST_F(ReaderTimestampIdxTest, TrueWhenZeroMessageStreamHasNoTimestamps) {
    const std::string path =
        write_minimal_streaming_osf(test_data_dir(), tmp_file("ts_idx_zero_msg.osf"));

    StreamingInfo modified;
    {
        Reader reader(path);
        auto streaming_info = reader.meta_store().get<StreamingInfo>();
        ASSERT_NE(streaming_info, nullptr);

        auto stream_stats = streaming_info->stream_stats();
        StreamStats empty_stats;
        empty_stats.stream_id = 424242;
        empty_stats.message_count = 0;
        stream_stats.emplace(empty_stats.stream_id, empty_stats);

        modified = StreamingInfo(streaming_info->chunks_info(), stream_stats);
    }

    rewrite_streaming_info(path, modified);

    Reader modified_reader(path);
    EXPECT_TRUE(modified_reader.has_timestamp_idx());
}

}  // namespace
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
