#include "ouster/osf/basics.h"

#include <gtest/gtest.h>

#include <fstream>

#include "fb_utils.h"
#include "osf_test.h"

namespace ouster {
namespace osf {
namespace {

class BasicsTest : public OsfTestWithData {};

TEST_F(BasicsTest, GetBlockSizeTest) {
    const std::string test_file_name =
        path_concat(test_data_dir(), "osfs/OS-1-128_v2.3.0_1024x10_lb_n3.osf");
    auto size = ouster::osf::file_size(test_file_name);
    char* buf = new char[size];
    std::fstream file_stream;
    file_stream.open(test_file_name, std::fstream::in | std::fstream::binary);
    file_stream.read(buf, size);
    EXPECT_EQ(ouster::osf::get_block_size((uint8_t*)buf), 60);
    delete[] buf;
}

}  // namespace
}  // namespace osf
}  // namespace ouster
