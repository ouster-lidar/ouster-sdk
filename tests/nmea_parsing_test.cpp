/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include "ouster/types.h"

using namespace ouster::sdk::core;

TEST(NmeaParsingTest, parse_sentence_test) {
    std::string nmea_sentence =
        "$GPRMC,210230,A,3855.4487,N,09446.0071,W,0.0,076.2,130495,003.8,E*69";

    double latitude = 0, longitude = 0;
    EXPECT_TRUE(parse_lat_long(nmea_sentence, latitude, longitude));

    EXPECT_DOUBLE_EQ(latitude, 38.924145);
    EXPECT_DOUBLE_EQ(longitude, -94.766785);
}
