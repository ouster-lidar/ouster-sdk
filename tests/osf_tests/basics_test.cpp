#include "ouster/osf/basics.h"

#include <gtest/gtest.h>

#include <fstream>
#include <vector>

#include "osf_test.h"
#include "ouster/osf/buffer.h"

namespace ouster {
namespace sdk {
namespace osf {
namespace {

class BasicsTest : public OsfTestWithData {};

TEST_F(BasicsTest, GetBlockSizeTest) {
    const std::string test_file_name =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");
    auto size = ouster::sdk::osf::file_size(test_file_name);
    char* buf = new char[size];
    std::fstream file_stream;
    file_stream.open(test_file_name, std::fstream::in | std::fstream::binary);
    file_stream.read(buf, size);
    OsfBuffer osf_buf;
    osf_buf.load_data((const uint8_t*)buf, size);
    EXPECT_EQ(ouster::sdk::osf::get_block_size(osf_buf), 60);
    delete[] buf;
}

}  // namespace
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
