/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/camera_model.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>

#include "ouster/core/camera_info.h"
#include "ouster/core/xyzlut.h"

using namespace ouster::sdk::core;

namespace {
// Wrap a camera_info body in the minimal valid metadata envelope.
CameraInfo camera_info_with(const std::string& fields) {
    return CameraInfo(R"({"camera_info": {"format": {"columns_per_frame": 4, )"
                      R"("pixels_per_column": 3, "fps": 30})" +
                      fields + "}}");
}
}  // namespace

TEST(CameraModel, RejectsNonPositiveFocalLength) {
    const CameraModel model(100.0f, 110.0f, 4.0f, 3.0f);
    EXPECT_FLOAT_EQ(model.fx, 100.0f);
    EXPECT_FLOAT_EQ(model.fy, 110.0f);
    EXPECT_FLOAT_EQ(model.cx, 4.0f);
    EXPECT_FLOAT_EQ(model.cy, 3.0f);

    EXPECT_THROW(CameraModel(0.0f, 100.0f, 0.0f, 0.0f), std::invalid_argument);
    EXPECT_THROW(CameraModel(100.0f, 0.0f, 0.0f, 0.0f), std::invalid_argument);
    EXPECT_THROW(CameraModel(-1.0f, 100.0f, 0.0f, 0.0f), std::invalid_argument);
    EXPECT_THROW(CameraModel(100.0f, -1.0f, 0.0f, 0.0f), std::invalid_argument);
    // NaN is not greater than zero either.
    EXPECT_THROW(CameraModel(std::numeric_limits<float>::quiet_NaN(), 100.0f, 0.0f, 0.0f),
                 std::invalid_argument);
}

TEST(CameraModel, EqualityIsPerField) {
    const CameraModel model(100.0f, 110.0f, 4.0f, 3.0f);
    EXPECT_EQ(model, CameraModel(100.0f, 110.0f, 4.0f, 3.0f));
    EXPECT_NE(model, CameraModel(101.0f, 110.0f, 4.0f, 3.0f));
    EXPECT_NE(model, CameraModel(100.0f, 111.0f, 4.0f, 3.0f));
    EXPECT_NE(model, CameraModel(100.0f, 110.0f, 5.0f, 3.0f));
    EXPECT_NE(model, CameraModel(100.0f, 110.0f, 4.0f, 2.0f));
}

TEST(CameraInfo, ModelIsAbsentByDefault) {
    const CameraInfo info;
    EXPECT_FALSE(static_cast<bool>(info.model));
}

TEST(CameraInfo, JsonRoundTripWithModel) {
    CameraInfo info;
    info.sn = 12345;
    info.format.columns_per_frame = 4;
    info.format.pixels_per_column = 3;
    info.format.fps = 30;
    info.model = CameraModel(100.0f, 110.0f, 2.0f, 1.5f);

    const auto roundtrip = CameraInfo(info.to_json_string());
    ASSERT_TRUE(static_cast<bool>(roundtrip.model));
    EXPECT_EQ(*roundtrip.model, *info.model);
    EXPECT_EQ(roundtrip.format, info.format);
    EXPECT_EQ(roundtrip, info);
}

TEST(CameraInfo, JsonRoundTripWithoutModel) {
    CameraInfo info;
    info.sn = 12345;
    info.format.columns_per_frame = 4;
    info.format.pixels_per_column = 3;

    // An absent model must stay absent rather than round-tripping into a
    // zero-valued one: the two mean different things (see the camera spec).
    const auto json = info.to_json_string();
    EXPECT_EQ(json.find("\"model\""), std::string::npos);

    const auto roundtrip = CameraInfo(json);
    EXPECT_FALSE(static_cast<bool>(roundtrip.model));
    EXPECT_EQ(roundtrip, info);
}

TEST(CameraInfo, ModelParticipatesInEquality) {
    CameraInfo without_model;
    without_model.format.columns_per_frame = 4;
    without_model.format.pixels_per_column = 3;

    auto with_model = without_model;
    with_model.model = CameraModel(100.0f, 100.0f, 2.0f, 1.5f);
    EXPECT_NE(with_model, without_model);

    auto other_model = without_model;
    other_model.model = CameraModel(50.0f, 100.0f, 2.0f, 1.5f);
    EXPECT_NE(with_model, other_model);

    auto same_model = without_model;
    same_model.model = CameraModel(100.0f, 100.0f, 2.0f, 1.5f);
    EXPECT_EQ(with_model, same_model);
}

TEST(CameraInfo, MetadataRejectsNonPositiveFocalLength) {
    const auto info = camera_info_with(R"(, "model": {"fx": 100.0, "fy": 100.0})");
    ASSERT_TRUE(static_cast<bool>(info.model));
    EXPECT_FLOAT_EQ(info.model->fx, 100.0f);

    // A missing or non-positive focal length is a metadata error, not a model
    // with fx == 0.
    EXPECT_THROW(camera_info_with(R"(, "model": {"fy": 100.0})"), std::runtime_error);
    EXPECT_THROW(camera_info_with(R"(, "model": {"fx": 0.0, "fy": 100.0})"), std::runtime_error);
    EXPECT_THROW(camera_info_with(R"(, "model": {"fx": -1.0, "fy": 1.0})"), std::runtime_error);
    EXPECT_THROW(camera_info_with(R"(, "model": 7)"), std::runtime_error);

    // Camera metadata carrying no model at all is still valid.
    const auto without_model = camera_info_with("");
    EXPECT_FALSE(static_cast<bool>(without_model.model));
    EXPECT_EQ(without_model.w(), 4u);
    EXPECT_EQ(without_model.h(), 3u);
    EXPECT_EQ(without_model.fps(), 30);
}

TEST(CameraInfo, XyzLutIsEmptyWithoutAModel) {
    CameraInfo info;
    info.format.columns_per_frame = 4;
    info.format.pixels_per_column = 3;

    // Without intrinsics there is no defensible way to project, and guessing a
    // focal length would yield a wrong cloud that looks entirely plausible, so
    // the LUT is empty and refuses to project a frame.
    const XYZLut empty(info);
    EXPECT_EQ(empty.w, 0u);
    EXPECT_EQ(empty.h, 0u);
    EXPECT_THROW(empty(img_t<uint32_t>::Zero(3, 4)), std::invalid_argument);

    info.model = CameraModel(100.0f, 100.0f, 2.0f, 1.5f);
    const XYZLut lut(info);
    EXPECT_EQ(lut.w, 4u);
    EXPECT_EQ(lut.h, 3u);
}

TEST(CameraInfo, XyzLutIsEmptyWithZeroDimensions) {
    CameraInfo info;
    info.model = CameraModel(100.0f, 100.0f, 2.0f, 1.5f);
    info.format.columns_per_frame = 4;
    info.format.pixels_per_column = 0;
    const XYZLut zero_height(info);
    EXPECT_EQ(zero_height.w, 0u);
    EXPECT_EQ(zero_height.h, 0u);

    info.format.columns_per_frame = 0;
    info.format.pixels_per_column = 3;
    const XYZLut zero_width(info);
    EXPECT_EQ(zero_width.w, 0u);
    EXPECT_EQ(zero_width.h, 0u);
}

TEST(CameraInfo, XyzLutRejectsModelMutatedToZeroFocalLength) {
    CameraInfo info;
    info.model = CameraModel(100.0f, 100.0f, 2.0f, 1.5f);
    info.format.columns_per_frame = 4;
    info.format.pixels_per_column = 3;
    info.model->fx = 0.0f;
    EXPECT_THROW(XYZLut{info}, std::invalid_argument);
}
