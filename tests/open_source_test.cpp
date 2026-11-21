/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/open_source.h"

#include "gtest/gtest.h"
#include "util.h"

#ifdef OUSTER_SENSOR
#include "ouster/sensor/sensor_frame_set_source.h"
#endif

#ifdef OUSTER_PCAP
#include "ouster/pcap/pcap_frame_set_source.h"
#endif

#ifdef OUSTER_OSF
#include "ouster/osf/osf_frame_set_source.h"
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
    std::string osf_file = data_dir + "/osfs/OS-0-128_v3.0.1_1024x10_20241017_141645.osf";
    EXPECT_NO_THROW(ouster::sdk::open_source(osf_file));
#endif
}

/**
 * Regression test for issue #721:
 * https://github.com/ouster-lidar/ouster-sdk/issues/721
 *
 * The PacketSourceOptions(FrameSetSourceOptions&) converting constructor
 * previously omitted config_timeout from its move-list. This had two effects:
 *   1. The caller's config_timeout value was silently discarded, so the packet
 *      source always fell back to its default.
 *   2. The source-side "set" flag was never cleared, so a later
 *      options.check() would throw "Parameter 'config_timeout' not supported".
 *      In SensorFrameSetSource this check ran after a joinable batcher thread
 *      had been assigned, so unwinding destroyed the joinable std::thread and
 *      called std::terminate() -- aborting the whole process with SIGABRT.
 */
TEST(OpenSourceTest, test_config_timeout_forwarded_from_frame_set_options) {
    ouster::sdk::FrameSetSourceOptions fopts;
    fopts.config_timeout = 12.0f;

    ouster::sdk::PacketSourceOptions popts(fopts);

    // The value must actually be forwarded to the packet source and
    // not silently reset to the default.
    EXPECT_TRUE(popts.config_timeout.was_set());
    EXPECT_FLOAT_EQ(popts.config_timeout.value_, 12.0f);

    // The source-side flag must be cleared so that option
    // validation no longer rejects config_timeout.
    EXPECT_NO_THROW(fopts.check("SensorFrameSetSource"));
}
