#include <gtest/gtest.h>

#include <algorithm>
#include <nonstd/optional.hpp>
#include <string>
#include <vector>

#include "common.h"
#include "osf_test.h"
#include "ouster/core/camera_frame.h"
#include "ouster/core/camera_info.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/osf_frame_set_source.h"
#include "ouster/osf/reader.h"

using ouster::sdk::core::CameraFrame;
using ouster::sdk::core::CameraInfo;
using ouster::sdk::core::CameraModel;
using ouster::sdk::core::fd_array;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::FrameSet;
using ouster::sdk::core::FrameSetSourceMetadata;
using ouster::sdk::core::FrameSetSourceMetadataSet;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::LidarInfo;
using ouster::sdk::core::metadata_from_json;

namespace ouster {
namespace sdk {
namespace osf {

class OsfFrameSetSourceTest : public osf::OsfTestWithDataAndFiles {};

TEST_F(OsfFrameSetSourceTest, frame_set_source_metadata) {
    // Create an OSF file with some metadata in it.
    const LidarInfo sinfo =
        metadata_from_json(path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    std::string output_osf_filename = tmp_file("frame_set_source_metadata.osf");
    Writer writer(output_osf_filename);
    FrameSetSourceMetadataSet metadata;
    metadata.entries.emplace("test_string", FrameSetSourceMetadata(std::string{"hello"}));
    metadata.entries.emplace("test_string_2", FrameSetSourceMetadata(std::string{"goodbye"}));
    writer.save(metadata);
    writer.close();

    // Read the source and check that the metadata is present and correct.
    // auto source = open_source(output_osf_filename);
    auto source = open_source(output_osf_filename);
    EXPECT_TRUE(source.has_metadata("test_string"));
    EXPECT_FALSE(source.has_metadata("non_existent"));

    EXPECT_EQ(source.metadata_keys(), (std::set<std::string>{"test_string", "test_string_2"}));

    EXPECT_EQ(source.metadata("test_string").as<std::string>(), "hello");
    EXPECT_EQ(source.metadata("test_string_2").as<std::string>(), "goodbye");

    unlink(output_osf_filename.c_str());
}

TEST_F(OsfFrameSetSourceTest, it_yields_empty_set_if_the_source_has_no_valid_frame) {
    std::string output_osf_filename = tmp_file("empty_frame_set_source.osf");
    std::string metadata_path = path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json");
    auto sinfo = std::make_shared<LidarInfo>(metadata_from_json(metadata_path));
    auto frame = std::make_shared<LidarFrame>(sinfo);
    frame->receive_timestamp() = 1000;
    frame->status() = 1;

    // modify the sensor info so that the source will have a mismatched number
    // of columns per packets versus the frames because of this, none of the
    // frames will be considered valid and the source should yield nullptrs
    // TODO perhaps we also want to consider that the writer should prevent this
    // in the first place
    sinfo->format.columns_per_packet = 10;
    Writer writer(output_osf_filename, {*sinfo, *sinfo});
    writer.save(FrameSet{frame, nullptr});
    writer.close();

    // default options, collate true
    auto source = open_source(output_osf_filename, {}, true);
    for (const auto& frame_set : source) {
        ASSERT_GT(frame_set.frames().size(), 0u);
        for (const auto& lf : frame_set.frames()) {
            EXPECT_EQ(lf, nullptr);
        }
    }

    // default options, collate false
    auto source2 = open_source(output_osf_filename, {}, false);
    for (const auto& frame_set : source2) {
        for (const auto& lf : frame_set.frames()) {
            EXPECT_EQ(lf, nullptr);
        }
    }

    unlink(output_osf_filename.c_str());
}

TEST_F(OsfFrameSetSourceTest, it_yields_empty_set_if_collation_msg_has_no_frame_ids) {
    std::string output_osf_filename = tmp_file("malformed_collation.osf");
    LidarInfo sinfo =
        metadata_from_json(path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));

    {
        Writer writer(output_osf_filename, {sinfo});
        CollationStreamMeta meta;
        writer.add_metadata(meta);
        uint32_t stream_id = meta.id();

        // Create a malformed CollationMsg (missing scan_ids)
        flatbuffers::FlatBufferBuilder fbb(1024);
        auto msg_off = ouster::sdk::osf::impl::gen::CreateCollationMsg(fbb, 0, 0, 0);
        fbb.FinishSizePrefixed(msg_off);

        std::vector<uint8_t> buf(fbb.GetBufferPointer(), fbb.GetBufferPointer() + fbb.GetSize());

        writer.save_message(stream_id, ts_t{100}, ts_t{100}, buf,
                            MetadataTraits<CollationStreamMeta>::type());
        writer.close();
    }

    auto source = open_source(output_osf_filename, {}, true);
    ASSERT_TRUE(source.is_collated());

    auto it = source.begin();
    auto end = source.end();
    ASSERT_TRUE(it != end);

    FrameSet set = *it;
    EXPECT_EQ(set.size(), 0u);

    unlink(output_osf_filename.c_str());
}

// A mixed lidar+camera collation must be written so that SDK versions predating
// camera support can still read the lidar streams. Those readers only consult
// the legacy `scan_ids` field and index it against their lidar-only sensor
// table, so `scan_ids` must (a) contain lidar slots only, (b) address them by
// per-lidar index (not the unified index), and (c) mark null lidar slots with
// the {0xFFFF, 0} sentinel they understand. Camera-aware readers instead use
// the unified `frame_ids` field. This locks that wire contract.
TEST_F(OsfFrameSetSourceTest, collation_msg_has_legacy_lidar_only_and_unified_frame_ids) {
    const LidarInfo linfo =
        metadata_from_json(path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));

    CameraInfo cinfo;
    cinfo.model = CameraModel(100.0f, 100.0f, 0.0f, 0.0f);

    auto make_camera_frame = [](uint64_t ts) {
        auto frame = std::make_shared<CameraFrame>();
        frame->w = 4;
        frame->h = 3;
        frame->add_field("RANGE", fd_array<uint32_t>(3, 4), FieldClass::PIXEL_FIELD);
        frame->timestamp()[0] = ts;
        frame->receive_timestamp()[0] = ts;
        return frame;
    };
    auto make_lidar_frame = [&linfo]() {
        // get_random_lidar_frame stamps valid status/timestamps so the frame can
        // participate in a collation (a frame with no valid receive timestamp is
        // rejected by the writer).
        return std::make_shared<LidarFrame>(get_random_lidar_frame(linfo));
    };

    std::string output_osf_filename = tmp_file("collation_wire_contract.osf");

    // Register camera *first* so the lidars' unified indices (1, 2) differ from
    // their per-lidar indices (0, 1); this is exactly the case a naive unified
    // encoding would get wrong for older readers.
    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> infos{
        std::make_shared<CameraInfo>(cinfo), std::make_shared<LidarInfo>(linfo),
        std::make_shared<LidarInfo>(linfo)};
    {
        Writer writer(output_osf_filename, infos);
        // Slots (registration order): [camera0, lidar0, lidar1]. Leave lidar1
        // null to exercise the null-slot sentinels on both sides.
        std::vector<std::shared_ptr<ouster::sdk::core::Frame>> frames{make_camera_frame(1000),
                                                                      make_lidar_frame(), nullptr};
        writer.save(FrameSet{std::move(frames)});
        writer.close();
    }

    // Find the (single) collation message and parse its raw flatbuffer.
    Reader reader(output_osf_filename);
    auto collation_meta = reader.meta_store().get<CollationStreamMeta>();
    ASSERT_NE(collation_meta, nullptr);
    const uint32_t collation_id = collation_meta->id();

    int collation_msgs = 0;
    for (const auto& m : reader.messages()) {
        if (m.id() != collation_id) {
            continue;
        }
        ++collation_msgs;
        const auto buf = m.buffer();
        const auto* cm = flatbuffers::GetSizePrefixedRoot<impl::gen::CollationMsg>(buf.data());

        // Legacy scan_ids: lidar slots only, addressed by per-lidar index, with
        // the null lidar slot as the {0xFFFF, 0} sentinel understood by readers
        // that predate camera support.
        const auto* scan_ids = cm->scan_ids();
        ASSERT_NE(scan_ids, nullptr);
        ASSERT_EQ(scan_ids->size(), 2u);
        EXPECT_EQ(scan_ids->Get(0)->sensor_id(), 0u);  // lidar0 -> per-lidar index 0
        EXPECT_EQ(scan_ids->Get(0)->scan_idx(), 0u);
        EXPECT_EQ(scan_ids->Get(1)->sensor_id(), LEGACY_INVALID_SCAN_ID.first);  // 0xFFFF
        EXPECT_EQ(scan_ids->Get(1)->scan_idx(), LEGACY_INVALID_SCAN_ID.second);  // 0

        // Unified frame_ids: one entry per collation slot, addressed by unified
        // index, with the null slot as UINT64_MAX.
        const auto* frame_ids = cm->frame_ids();
        ASSERT_NE(frame_ids, nullptr);
        ASSERT_EQ(frame_ids->size(), 3u);
        EXPECT_EQ(frame_ids->Get(0)->sensor_id(), 0u);  // camera0 (unified index 0)
        EXPECT_EQ(frame_ids->Get(0)->scan_idx(), 0u);
        EXPECT_EQ(frame_ids->Get(1)->sensor_id(), 1u);  // lidar0 (unified index 1)
        EXPECT_EQ(frame_ids->Get(1)->scan_idx(), 0u);
        EXPECT_EQ(frame_ids->Get(2)->sensor_id(), 2u);  // lidar1 (unified index 2, null)
        EXPECT_EQ(frame_ids->Get(2)->scan_idx(), INVALID_COLLATION_FRAME_INDEX);
    }
    EXPECT_EQ(collation_msgs, 1);

    // The camera-aware reader round-trips the full collation via frame_ids.
    auto source = open_source(output_osf_filename, {}, true);
    ASSERT_TRUE(source.is_collated());
    std::vector<FrameSet> sets;
    for (auto fs : source) {
        sets.push_back(std::move(fs));
    }
    ASSERT_EQ(sets.size(), 1u);
    ASSERT_EQ(sets[0].size(), 3u);
    EXPECT_NE(std::dynamic_pointer_cast<CameraFrame>(sets[0][0]), nullptr);
    EXPECT_NE(std::dynamic_pointer_cast<LidarFrame>(sets[0][1]), nullptr);
    EXPECT_EQ(sets[0][2], nullptr);

    unlink(output_osf_filename.c_str());
}

// FrameSetSourceOptions::field_names must filter camera streams just like lidar
// streams, and a requested field a camera frame does not have is an error
// (reported through the error handler, dropping the frame), as it is for lidar.
TEST_F(OsfFrameSetSourceTest, camera_stream_honors_field_names) {
    CameraInfo cinfo;
    cinfo.model = CameraModel(100.0f, 100.0f, 0.0f, 0.0f);

    std::string output_osf_filename = tmp_file("camera_field_names.osf");
    {
        std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> infos{
            std::make_shared<CameraInfo>(cinfo)};
        Writer writer(output_osf_filename, infos);
        for (uint64_t ts : {1000u, 2000u}) {
            CameraFrame frame;
            frame.w = 4;
            frame.h = 3;
            frame.add_field("RANGE", fd_array<uint32_t>(3, 4), FieldClass::PIXEL_FIELD);
            frame.add_field("RGB", fd_array<uint8_t>(3, 4, 3), FieldClass::PIXEL_FIELD);
            frame.timestamp()[0] = ts;
            frame.receive_timestamp()[0] = ts;
            writer.save(0, frame, ts_t{ts});
        }
        writer.close();
    }

    std::vector<std::string> warnings;
    auto read_fields = [&](const nonstd::optional<std::vector<std::string>>& field_names) {
        auto source = open_source(
            output_osf_filename,
            [&](ouster::sdk::FrameSetSourceOptions& options) {
                options.field_names = field_names;
                options.error_handler = [&](ouster::sdk::core::Severity /*severity*/,
                                            const std::string& msg) { warnings.push_back(msg); };
            },
            false);
        std::vector<std::vector<std::string>> fields;
        for (const auto& frame_set : source) {
            for (const auto& frame : frame_set.frames()) {
                auto camera = std::dynamic_pointer_cast<CameraFrame>(frame);
                if (!camera) {
                    continue;
                }
                std::vector<std::string> names;
                for (const auto& field : camera->fields()) {
                    names.push_back(field.first);
                }
                std::sort(names.begin(), names.end());
                fields.push_back(names);
            }
        }
        return fields;
    };

    using Names = std::vector<std::string>;
    EXPECT_EQ(read_fields(nonstd::nullopt),
              (std::vector<Names>{{"RANGE", "RGB"}, {"RANGE", "RGB"}}));
    EXPECT_EQ(read_fields(Names{"RGB"}), (std::vector<Names>{{"RGB"}, {"RGB"}}));
    EXPECT_EQ(read_fields(Names{}), (std::vector<Names>{{}, {}}));
    EXPECT_TRUE(warnings.empty());

    EXPECT_TRUE(read_fields(Names{"RGB", "NOT_A_FIELD"}).empty());
    ASSERT_EQ(warnings.size(), 2u);
    for (const auto& warning : warnings) {
        EXPECT_NE(warning.find("NOT_A_FIELD"), std::string::npos) << warning;
    }

    unlink(output_osf_filename.c_str());
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
