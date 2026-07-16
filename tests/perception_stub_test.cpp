/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <vector>

#include "ouster/perception/detection_engine.h"

using ouster::sdk::perception::ClassicDetectionConfig;
using ouster::sdk::perception::DetectionConfig;
using ouster::sdk::perception::DetectionEngine;

TEST(DetectionEngineTest, test_config_typeid_sanity) {
    DetectionConfig base_config{};
    ClassicDetectionConfig classic_config{};

    const DetectionConfig& ref_base = base_config;
    const DetectionConfig& ref_classic = classic_config;
    EXPECT_FALSE(typeid(ref_classic) == typeid(ref_base));

    EXPECT_TRUE(typeid(ref_classic) == typeid(ClassicDetectionConfig));
    EXPECT_FALSE(typeid(ref_classic) == typeid(DetectionConfig));
    EXPECT_FALSE(typeid(ref_base) == typeid(ClassicDetectionConfig));
    EXPECT_TRUE(typeid(ref_base) == typeid(DetectionConfig));
}

TEST(DetectionEngineTest, test_stub_throws_correctly) {
    std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>> infos;

    EXPECT_THROW(DetectionEngine::create(infos), std::runtime_error);
}
