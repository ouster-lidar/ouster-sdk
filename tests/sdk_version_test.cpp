/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <cstring>
#include <iostream>
#include <string>

#include "ouster/core/impl/build.h"

namespace {
const char* str_or_null(const char* s) {
    return s ? s : "(null)";
}
}  // namespace

TEST(SdkVersion, VersionStringsArePopulated) {
    using namespace ouster::sdk;
    std::cout << "ouster::sdk (build.h / generated build.cpp):\n"
              << "  SDK_VERSION:      " << str_or_null(SDK_VERSION) << "\n"
              << "  SDK_VERSION_FULL: " << str_or_null(SDK_VERSION_FULL) << "\n"
              << "  BUILD_HASH:       " << str_or_null(BUILD_HASH) << "\n"
              << "  BUILD_BRANCH:     " << str_or_null(BUILD_BRANCH) << "\n"
              << "  BUILD_TYPE:       " << str_or_null(BUILD_TYPE) << "\n"
              << "  BUILD_SYSTEM:     " << str_or_null(BUILD_SYSTEM) << "\n"
              << std::flush;

    ASSERT_NE(SDK_VERSION, nullptr);
    EXPECT_GT(std::strlen(SDK_VERSION), 0u);

    ASSERT_NE(SDK_VERSION_FULL, nullptr);
    EXPECT_GT(std::strlen(SDK_VERSION_FULL), 0u);

    ASSERT_NE(BUILD_TYPE, nullptr);
    ASSERT_NE(BUILD_SYSTEM, nullptr);
}

#ifdef OUSTER_SDK_VERSION_EXPECTED
TEST(SdkVersion, VersionMatchesBuildConfiguration) {
    std::cout << "OUSTER_SDK_VERSION_EXPECTED: " << OUSTER_SDK_VERSION_EXPECTED << "\n"
              << std::flush;
    EXPECT_STREQ(ouster::sdk::SDK_VERSION, OUSTER_SDK_VERSION_EXPECTED);

    std::string full(ouster::sdk::SDK_VERSION_FULL);
    EXPECT_NE(full.find(OUSTER_SDK_VERSION_EXPECTED), std::string::npos);
}
#endif
