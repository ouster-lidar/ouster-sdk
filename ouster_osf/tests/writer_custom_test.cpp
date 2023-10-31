/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <string>

#include "common.h"
#include "osf_test.h"
#include "ouster/osf/file.h"
#include "ouster/osf/reader.h"
#include "ouster/osf/writer.h"

namespace ouster {
namespace osf {

class WriterCustomTest : public osf::OsfTestWithDataAndFiles {};

class MyNewMetaInfo : public MetadataEntryHelper<MyNewMetaInfo> {
   public:
    explicit MyNewMetaInfo(const std::string& text) : text_(text) {}

    const std::string& text() const { return text_; }

    // Pack to byte array
    std::vector<uint8_t> buffer() const final {
        return {text_.begin(), text_.end()};
    }

    // UnPack from byte array
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf) {
        std::string s(buf.begin(), buf.end());
        return std::make_unique<MyNewMetaInfo>(s);
    }

    // Custom view for nice to_string() output
    std::string repr() const override { return "text: '" + text_ + "'"; }

   private:
    std::string text_;
};

template <>
struct MetadataTraits<MyNewMetaInfo> {
    static const std::string type() { return "ouster/v1/MyNewSuperMetaInfo"; }
};

// TODO[pb]: Define a StreamTagHelper for just dummy types/tags for use in
//           custom stream definitions
class YoStreamMeta : public MetadataEntryHelper<YoStreamMeta> {
   public:
    YoStreamMeta() {}
    std::vector<uint8_t> buffer() const final { return {}; };
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>&) {
        return std::make_unique<YoStreamMeta>();
    }
};

template <>
struct MetadataTraits<YoStreamMeta> {
    static const std::string type() { return "ouster/v1/YoStream"; }
};

// Message object of YoStream
struct yo {
    uint8_t a;
};

// Define custom stream with the message type `yo`
class YoStream : public MessageStream<YoStreamMeta, yo> {
   public:
    YoStream(Writer& writer) : writer_{writer}, meta_{} {
        stream_meta_id_ = writer_.addMetadata(meta_);
    };

    // Boilerplate for writer
    void save(const ouster::osf::ts_t ts, const obj_type& yo_obj) {
        const auto& msg_buf = make_msg(yo_obj);
        writer_.saveMessage(meta_.id(), ts, msg_buf);
    }

    // Pack yo message into buffer
    std::vector<uint8_t> make_msg(const obj_type& yo_obj) { return {yo_obj.a}; }

    // UnPack yo message from buffer
    static std::unique_ptr<obj_type> decode_msg(const std::vector<uint8_t>& buf,
                                                const meta_type&,
                                                const MetadataStore&) {
        auto y = std::make_unique<yo>();
        y->a = buf[0];
        return y;
    }

   private:
    Writer& writer_;

    meta_type meta_;

    uint32_t stream_meta_id_{0};
};

TEST_F(WriterCustomTest, WriteCustomMsgExample) {
    std::string output_osf_filename = tmp_file("writer_new_meta_info_msg.osf");

    // Create OSF v2 Writer
    osf::Writer writer(output_osf_filename, "Yo Example");

    // Create LidarSensor record
    writer.addMetadata<MyNewMetaInfo>("Happy New Year!");

    // Create stream for `yo` objects
    auto yo_stream = writer.createStream<YoStream>();

    uint8_t yo_cnt = 0;
    while (yo_cnt < 100) {
        // `yo` object
        yo y;
        y.a = (uint8_t)yo_cnt;

        // Save `yo` object into stream
        ts_t ts{yo_cnt * 10};
        yo_stream.save(ts, y);

        ++yo_cnt;
    }

    writer.close();

    OsfFile file(output_osf_filename);
    osf::Reader reader(file);

    // Read all messages from OSF file
    int msg_cnt = 0;
    for (const auto m : reader.messages()) {
        // Decoding messages
        if (m.is<YoStream>()) {
            auto y = *m.decode_msg<YoStream>();
            // Check `yo` msgs are the same and in the same order as written
            EXPECT_EQ(msg_cnt, y.a);
            // std::cout << "yo = " << (int)y.a << std::endl;
        }
        ++msg_cnt;
    }
    EXPECT_EQ(100, msg_cnt);

    auto my_metas = reader.meta_store().find<MyNewMetaInfo>();
    EXPECT_EQ(1, my_metas.size());

    // Get MyNewMetaInfo metadata
    auto my_meta = my_metas.begin()->second;
    EXPECT_EQ(my_meta->text(), "Happy New Year!");

    std::cout << "my_meta = " << my_meta->to_string() << std::endl;
    // std::cout << "output = " << output_osf_filename << std::endl;
}

}  // namespace osf
}  // namespace ouster