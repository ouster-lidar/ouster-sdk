#ifndef OUSTER_ROS_MOTION_CORRECTION_H_
#define OUSTER_ROS_MOTION_CORRECTION_H_

#include <kindr/minimal/quat-transformation.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <list>
#include "ouster_ros/point_os1.h"

namespace ouster_ros {
namespace OS1 {

typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;

template <typename Type>
using AlignedList = std::list<Type, Eigen::aligned_allocator<Type>>;

class MotionCorrection {
 public:
  MotionCorrection(const size_t max_pointcloud_queue_length,
                   const bool output_reflectivity_as_intensity = false,
                   const bool remove_old_transformations = true);

  void addTransformation(
      const ros::Time& stamp,
      const kindr::minimal::QuatTransformation& transformation);

  void addPointCloudToQueue(
      const std::shared_ptr<pcl::PointCloud<PointOS1>>& pointcloud_ptr);

  bool processNextQueuedPointcloud(pcl::PointCloud<pcl::PointXYZI>* pointcloud);

 private:
  enum class InterpolationStatus {
    NOT_ENOUGH_POINTS,
    BEFORE_FIRST,
    MATCHED,
    AFTER_LAST
  };

  struct TransformationStamped {
    uint64_t stamp;
    Transformation transformation;
  };

  // small private functor class
  class Interpolator {
   public:
    Interpolator(const TransformationStamped& transformation_before,
                 const TransformationStamped& transformation_after)
        : before_stamp_(transformation_before.stamp),
          delta_stamp_(static_cast<float>(transformation_after.stamp -
                                          transformation_before.stamp)),
          transformation_before_(transformation_before.transformation),
          delta_vector_((transformation_before.transformation.inverse() *
                         transformation_after.transformation)
                            .log()) {}

    Transformation operator()(const uint64_t stamp) const {
      const float t_delta_ratio =
          static_cast<float>(stamp - before_stamp_) / delta_stamp_;
      return transformation_before_ *
             Transformation::exp(t_delta_ratio * delta_vector_);
    }

   private:
    uint64_t before_stamp_;
    double delta_stamp_;
    Transformation transformation_before_;
    Transformation::Vector6 delta_vector_;
  };

  InterpolationStatus transformPoints(
      const pcl::PointCloud<PointOS1>& pointcloud_in,
      pcl::PointCloud<pcl::PointXYZI>* pointcloud_out);

  const size_t max_pointcloud_queue_length_;
  const bool output_reflectivity_as_intensity_;
  const bool remove_old_transformations_;

  AlignedList<std::shared_ptr<pcl::PointCloud<PointOS1>>> pointcloud_list_;
  AlignedList<TransformationStamped> transformation_list_;
  AlignedList<TransformationStamped>::const_iterator point_it_;
  AlignedList<TransformationStamped>::const_iterator cloud_it_;
};

}  // namespace OS1
}  // namespace ouster_ros

#endif  // OUSTER_ROS_MOTION_CORRECTION_H_
