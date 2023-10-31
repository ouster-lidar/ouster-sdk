/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/pcap_source.h"

#include <gtest/gtest.h>

#include "osf_test.h"
#include "ouster/osf/pcap_source.h"
#include "ouster/types.h"

namespace ouster {
namespace osf {
namespace {

class OsfPcapSourceTest : public OsfTestWithData {};

// TODO[pb]: Remove this test and PcapRawSource since it's not matching of what
// we have in the Python
TEST_F(OsfPcapSourceTest, ReadLidarScansAndImus) {
    std::string pcap_file = path_concat(
        test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10_lb_n3.pcap");
    std::string meta_file =
        path_concat(test_data_dir(), "pcaps/OS-1-128_v2.3.0_1024x10.json");

    PcapRawSource pcap_source{pcap_file};

    auto info = sensor::metadata_from_json(meta_file);

    int ls_cnt = 0;

    pcap_source.addLidarDataHandler(
        7502, info, [&ls_cnt](const osf::ts_t, const LidarScan&) { ls_cnt++; });

    pcap_source.runAll();

    EXPECT_EQ(2, ls_cnt);
}

}  // namespace

}  // namespace osf
}  // namespace ouster
