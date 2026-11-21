/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Wall segmentation: WallSegEngine
 *
 * WallSegEngine::update() marks, per pixel, the returns of a frame that stand
 * on a real wall: a surface normal near horizontal is only a candidate, and
 * the candidates together have to stand on one continuous vertical surface
 * large enough to be a wall rather than foliage or clutter. plumb_walls()
 * reads the marked pixels back as points and votes with them.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
class FrameSet;
}  // namespace core

namespace algorithm {

/// @brief Configuration parameters for the wall segmentation engine.
struct OUSTER_API_CLASS WallSegConfig {
    /// Rotation-only alignment applied to every frame's cloud before the
    /// mask is taken, e.g. built from plumb_imu(). Identity when the frames
    /// are already level. Nothing on the frame is rotated; the mask is
    /// simply taken in the aligned frame.
    core::Matrix4dR gravity_rotation = core::Matrix4dR::Identity();
};

/**
 * @class WallSegEngine
 * @brief Wall segmentation engine for classifying wall points in frames.
 *
 * update() annotates every valid frame of a FrameSet with a per-pixel
 * mask, "WALL" for the first return and "WALL2" for the second when the
 * frame carries one, uint8 with 1 = wall and 0 = not.
 *
 * Any frame carrying a RANGE image is segmented, lidar or camera: a lidar
 * contributes its staggered returns, a depth camera its single unstaggered
 * one. A color-only camera has no geometry to stand a wall on and is logged
 * and skipped rather than failing the set.
 *
 * A pixel is wall only when its surface normal is near horizontal and it
 * stands on one continuous vertical surface large enough and long enough to
 * be a wall. Foliage, a parked car, a fence post or plain noise present
 * near-horizontal normals in numbers and answer 0; a facade answers 1. Dual
 * returns are supported, so a wall seen by both stands on one denser patch
 * rather than on two thinner ones.
 *
 * Coordinate frame: each frame's cloud is dewarped by its per-column poses
 * taken relative to its first valid column (intra-frame motion only, a
 * single-pose frame sharing that one pose across every pixel) and then
 * rotated by WallSegConfig::gravity_rotation. Normals are tested for
 * "near-horizontal" in that rotated frame, so this rotation is what tells
 * the engine which way is up.
 *
 * The engine never reads the IMU itself. Leave gravity_rotation at identity
 * when the frames are already level; otherwise compute the alignment before
 * calling update() and pass it in the config. plumb_imu() gives a gravity
 * direction that core::get_rot_matrix_to_align_to_gravity() turns into such
 * a rotation; plumb_walls() forwards its own gravity_rotation argument here
 * the same way.
 *
 * A frame that already carries "NORMALS" (and "NORMALS2") fields in world
 * coordinates is taken at its word rather than estimated again.
 */
class OUSTER_API_CLASS WallSegEngine {
   public:
    /**
     * @brief Create a WallSegEngine with the given configuration.
     *
     * @param[in] config Configuration parameters for the engine.
     * @return A unique pointer to the created WallSegEngine instance.
     */
    OUSTER_API_FUNCTION
    static std::unique_ptr<WallSegEngine> create(const WallSegConfig& config = {});

    OUSTER_API_FUNCTION
    virtual ~WallSegEngine() = default;

    /**
     * @brief The WALL masks of every valid frame, without annotating any.
     *
     * Indexed like the set: entry i holds frame i's masks, one per return it
     * carries ("WALL" first, "WALL2" second), uint8 with 1 = wall and 0 =
     * not. A null slot, or a sensor that cannot be segmented (logged rather
     * than failing the set), gets an empty entry. Sensors are segmented
     * concurrently and independently. update() is this plus writing the
     * masks into fields.
     *
     * @param[in] frames FrameSet to segment; nothing on it is modified.
     * @return One vector of masks per slot of @p frames.
     */
    OUSTER_API_FUNCTION
    virtual std::vector<std::vector<core::img_t<uint8_t>>> compute_walls(
        const core::FrameSet& frames) const = 0;

    /**
     * @brief Run wall segmentation on a FrameSet, annotating each frame in
     *        place with "WALL" (and "WALL2") fields.
     *
     * compute_walls() written back: existing WALL fields are replaced, a
     * return the frame does not carry is left without one rather than given
     * a mask of zeros, and a sensor that could not be segmented is left as it
     * was.
     *
     * @param[in,out] frames FrameSet to annotate in place.
     */
    OUSTER_API_FUNCTION
    virtual void update(core::FrameSet& frames) = 0;

   protected:
    WallSegEngine() = default;
};

}  // namespace algorithm
}  // namespace sdk
}  // namespace ouster
