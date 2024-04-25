#include "ouster/osf/meta_streaming_info.h"

#include <gtest/gtest.h>

#include <fstream>

#include "fb_utils.h"
#include "osf_test.h"
#include "ouster/osf/basics.h"

namespace ouster {
namespace osf {
namespace {

class MetaStreamingInfoTests : public OsfTestWithData {};

/// @todo move this to a better place
TEST_F(MetaStreamingInfoTests, StreamingPrintTests) {
    ChunkInfo data = {1, 2, 3};
    EXPECT_EQ(to_string(data),
              "{offset = 1, stream_id = 2, message_count = 3}");

    ts_t t(5678L);
    StreamStats data2(4, t, 6);
    EXPECT_EQ(to_string(data2),
              "{stream_id = 4, start_ts = 5678, end_ts = 5678,"
              " message_count = 1, message_avg_size = 6}");
}

}  // namespace
}  // namespace osf
}  // namespace ouster
