#include "ouster/point_viz.h"

#include <gtest/gtest.h>

using namespace ouster::viz;

TEST(PointViz, window_coordinates_to_world_coordinates) {
    WindowCtx ctx;
    ctx.window_width = ctx.viewport_width = 600;
    ctx.window_height = ctx.viewport_height = 400;
    auto world = ctx.normalized_coordinates(0, 0);
    EXPECT_DOUBLE_EQ(world.first, -1.5);
    EXPECT_DOUBLE_EQ(world.second, 1);

    // not meant to check out-of-bounds :)
    auto world2 = ctx.normalized_coordinates(-600, 200);
    EXPECT_DOUBLE_EQ(world2.first, -4.5);
    EXPECT_DOUBLE_EQ(world2.second, 0);
}

TEST(PointViz, window_coordinates_to_image_pixel) {
    WindowCtx ctx;
    ctx.window_width = ctx.viewport_width = 400;
    ctx.window_height = ctx.viewport_height = 300;
    Image img;
    auto pixel = img.window_coordinates_to_image_pixel(ctx, 0, 0);
    EXPECT_FALSE(pixel.has_value());  // by default, images have no width/height

    constexpr int w = 4;
    constexpr int h = 3;
    float img_data[w * h];
    img.set_image(w, h, img_data);
    img.set_position(-1.3333333333, 1.3333333333, -1, 1);
    pixel =
        img.window_coordinates_to_image_pixel(ctx, 0, ctx.window_height - 1);
    EXPECT_EQ(pixel->first, 0);
    EXPECT_EQ(pixel->second, 2);

    pixel = img.window_coordinates_to_image_pixel(ctx, 0, 0);
    EXPECT_EQ(pixel->first, 0);
    EXPECT_EQ(pixel->second, 0);

    pixel = img.window_coordinates_to_image_pixel(ctx, ctx.window_width - 1, 0);
    EXPECT_EQ(pixel->first, 3);
    EXPECT_EQ(pixel->second, 0);

    pixel = img.window_coordinates_to_image_pixel(ctx, ctx.window_width - 1,
                                                  ctx.window_height - 1);
    EXPECT_EQ(pixel->first, 3);
    EXPECT_EQ(pixel->second, 2);
}

TEST(PointViz, image_pixel_to_window_coordinates) {
    WindowCtx ctx;
    ctx.window_width = ctx.viewport_width = 400;
    ctx.window_height = ctx.viewport_height = 300;
    Image img;

    constexpr int w = 4;
    constexpr int h = 3;
    float img_data[w * h];
    img.set_image(w, h, img_data);
    img.set_position(-1.3333333333, 1.3333333333, -1, 1);
    auto pixel =
        img.window_coordinates_to_image_pixel(ctx, 0, ctx.window_height - 1);
    EXPECT_EQ(pixel->first, 0);
    EXPECT_EQ(pixel->second, 2);

    auto pixel_size = img.pixel_size(ctx);
    EXPECT_FLOAT_EQ(pixel_size.first, 100);
    EXPECT_FLOAT_EQ(pixel_size.second, 100);

    auto window_pixel =
        img.image_pixel_to_window_coordinates(ctx, pixel->first, pixel->second);
    EXPECT_FLOAT_EQ(window_pixel.first, 50);
    EXPECT_FLOAT_EQ(window_pixel.second, 250);
}
