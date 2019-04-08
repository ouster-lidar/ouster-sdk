#include "ouster_ros/motion_correction.h"
#include <pcl/common/transforms.h>

namespace ouster_ros {
namespace OS1 {

MotionCorrection::MotionCorrection(const size_t max_pointcloud_queue_length,
                                   const bool output_reflectivity_as_intensity,
                                   const bool remove_old_transformations)
    : max_pointcloud_queue_length_(max_pointcloud_queue_length),
      output_reflectivity_as_intensity_(output_reflectivity_as_intensity),
      remove_old_transformations_(remove_old_transformations) {}

void MotionCorrection::addTransformation(
    const ros::Time& stamp,
    const kindr::minimal::QuatTransformation& transformation) {
  const uint64_t stamp_us = stamp.toNSec() / 1000ul;

  if (!transformation_list_.empty() &&
      stamp_us < transformation_list_.back().stamp) {
    ROS_ERROR_STREAM(
        "Detected jump backwards in time, resetting transformation list");
    transformation_list_.clear();
  }

  TransformationStamped transformation_stamped;
  transformation_stamped.transformation = transformation.cast<float>();
  transformation_stamped.stamp = stamp_us;
  transformation_list_.push_back(transformation_stamped);

  if (transformation_list_.size() == 1) {
    point_it_ = transformation_list_.begin();
    cloud_it_ = transformation_list_.begin();
  }
}

void MotionCorrection::addPointCloudToQueue(
    const std::shared_ptr<pcl::PointCloud<PointOS1>>& pointcloud_ptr) {
  if (!pointcloud_list_.empty() &&
      pointcloud_ptr->header.stamp < pointcloud_list_.back()->header.stamp) {
    ROS_ERROR_STREAM(
        "Detected jump backwards in time, resetting pointcloud list");
    pointcloud_list_.clear();
  }

  if (pointcloud_list_.size() > max_pointcloud_queue_length_) {
    ROS_WARN_STREAM("There are now over "
                    << max_pointcloud_queue_length_
                    << " pointclouds waiting to be transformed, "
                       "removing oldest.");
    pointcloud_list_.pop_front();
  }

  pointcloud_list_.push_back(pointcloud_ptr);
}

bool MotionCorrection::processNextQueuedPointcloud(
    pcl::PointCloud<pcl::PointXYZI>* pointcloud) {
  while (!pointcloud_list_.empty()) {
    const InterpolationStatus status =
        transformPoints(*pointcloud_list_.front(), pointcloud);

    if (status == InterpolationStatus::MATCHED) {
      pointcloud_list_.pop_front();
      return true;
    }
    if ((status == InterpolationStatus::AFTER_LAST) ||
        (status == InterpolationStatus::NOT_ENOUGH_POINTS)) {
      return false;
    }
    if (status == InterpolationStatus::BEFORE_FIRST) {
      pointcloud_list_.pop_front();
    }
  }
  return false;
}

MotionCorrection::InterpolationStatus MotionCorrection::transformPoints(
    const pcl::PointCloud<PointOS1>& pointcloud_in,
    pcl::PointCloud<pcl::PointXYZI>* pointcloud_out) {
  pointcloud_out->header = pointcloud_in.header;
  pointcloud_out->reserve(pointcloud_in.size());

  ros::Time input_time;
  const uint64_t first_point_stamp =
      pointcloud_in.header.stamp + pointcloud_in.front().t;
  const uint64_t last_point_stamp =
      pointcloud_in.header.stamp + pointcloud_in.back().t;
  pointcloud_out->header.stamp =
      pointcloud_in.header.stamp + (pointcloud_in.front().t +
                                    pointcloud_in.back().t) /
                                       2;

  if (transformation_list_.size() < 2) {
    return InterpolationStatus::NOT_ENOUGH_POINTS;
  }
  if (transformation_list_.front().stamp > first_point_stamp) {
    return InterpolationStatus::BEFORE_FIRST;
  }
  if (transformation_list_.back().stamp < last_point_stamp) {
    return InterpolationStatus::AFTER_LAST;
  }

  while (cloud_it_->stamp < pointcloud_out->header.stamp) {
    ++cloud_it_;
  }

  Interpolator interpolator(*std::prev(cloud_it_), *cloud_it_);
  const Transformation transformation_pointcloud_inv =
      interpolator(pointcloud_out->header.stamp).inverse();

  AlignedList<TransformationStamped>::const_iterator previous_point_it =
      cloud_it_;
  uint64_t previous_point_stamp = 0ul;
  Eigen::Affine3f pcl_transform;

  for (const PointOS1& point : pointcloud_in) {
    const uint64_t point_stamp =
        pointcloud_in.header.stamp + point.t;

    if (point_stamp != previous_point_stamp) {
      while (point_it_->stamp < point_stamp) {
        ++point_it_;
      }

      if (previous_point_it != point_it_) {
        interpolator = Interpolator(*std::prev(point_it_), *point_it_);
        point_it_ = previous_point_it;
      }

      const Transformation delta_transforamtion =
          transformation_pointcloud_inv * interpolator(point_stamp);

      pcl_transform.matrix() = delta_transforamtion.getTransformationMatrix();
    }

    pcl::PointXYZI output_point;
    output_point.x = point.x;
    output_point.y = point.y;
    output_point.z = point.z;

    if (output_reflectivity_as_intensity_) {
      output_point.intensity = point.reflectivity;
    } else {
      output_point.intensity = point.intensity;
    }

    pointcloud_out->push_back(pcl::transformPoint(output_point, pcl_transform));
  }
  if (remove_old_transformations_) {
    while (transformation_list_.begin() != std::prev(point_it_)) {
      transformation_list_.pop_front();
    }
  }

  return InterpolationStatus::MATCHED;
}

}  // namespace OS1
}  // namespace ouster_ros
