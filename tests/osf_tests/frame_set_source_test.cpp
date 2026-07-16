#include <gtest/gtest.h>

#include "common.h"
#include "osf_test.h"
#include "ouster/core/open_source.h"
#include "ouster/core/types.h"
#include "ouster/osf/collation_stream.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/osf_frame_set_source.h"

using ouster::sdk::core::FrameSet;
using ouster::sdk::core::FrameSetSourceMetadata;
using ouster::sdk::core::FrameSetSourceMetadataSet;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::metadata_from_json;
using ouster::sdk::core::SensorInfo;

namespace ouster {
namespace sdk {
namespace osf {

class OsfFrameSetSourceTest : public osf::OsfTestWithDataAndFiles {};

TEST_F(OsfFrameSetSourceTest, frame_set_source_metadata) {
    // Create an OSF file with some metadata in it.
    const SensorInfo sinfo =
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
    SensorInfo sinfo =
        metadata_from_json(path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json"));
    auto frame = std::make_shared<LidarFrame>(sinfo);
    frame->packet_timestamp() = 1000;
    frame->status() = 1;

    // modify the sensor info so that the source will have a mismatched number
    // of columns per packets versus the frames because of this, none of the
    // frames will be considered valid and the source should yield nullptrs
    // TODO perhaps we also want to consider that the writer should prevent this
    // in the first place
    sinfo.format.columns_per_packet = 10;
    Writer writer(output_osf_filename, {sinfo, sinfo});
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
    SensorInfo sinfo =
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

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
