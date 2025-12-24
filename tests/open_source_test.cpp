/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/open_source.h"

#include "gtest/gtest.h"
#include "util.h"

#ifdef OUSTER_SENSOR
#include "ouster/sensor_scan_source.h"
#endif

#ifdef OUSTER_PCAP
#include "ouster/pcap_scan_source.h"
#endif

#ifdef OUSTER_OSF
#include "ouster/osf/osf_scan_source.h"
#endif

/**
 * Most open source tests are already implemented in python, here we are testing
 * some C++ specific behaviours.
 */

TEST(OpenSourceTest, test_open_source_builders_availability) {
#ifdef OUSTER_SENSOR
    // testing implementation details here, tsk tsk
    const auto& builders = ouster::sdk::impl::get_builders();
    EXPECT_NE(builders.find(ouster::sdk::core::IoType::SENSOR), builders.end());
#endif

    auto data_dir = getenvs("DATA_DIR");

#ifdef OUSTER_PCAP
    std::string pcap_file = data_dir + "/pcaps/OS-0-128-U1_v2.3.0_1024x10.pcap";
    EXPECT_NO_THROW(ouster::sdk::open_source(pcap_file));
#endif

#ifdef OUSTER_OSF
    std::string osf_file =
        data_dir + "/osfs/OS-0-128_v3.0.1_1024x10_20241017_141645.osf";
    EXPECT_NO_THROW(ouster::sdk::open_source(osf_file));
#endif
}
