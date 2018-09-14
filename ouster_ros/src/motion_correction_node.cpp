#include <memory>

#include <minkindr_conversions/kindr_tf.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "ouster_ros/motion_correction.h"

namespace ouster_ros {
namespace OS1 {

class MotionCorrectionNode {
 public:
  void tfTimerCallback(const ros::TimerEvent&) {
    static ros::Time prev_tf_stamp;
    tf::StampedTransform tf_transform;

    ros::Time latest_time;
    std::string error_string;
    tf_listener_.getLatestCommonTime(tf_odom_frame_, tf_sensor_frame_,
                                     latest_time, &error_string);
    if (!tf_listener_.canTransform(tf_sensor_frame_, tf_odom_frame_,
                                   latest_time, &error_string)) {
      ROS_ERROR_STREAM_THROTTLE(1, "Error getting TF transform from "
                                       << tf_odom_frame_ << " to "
                                       << tf_sensor_frame_ << ": "
                                       << error_string);
    } else if (latest_time != prev_tf_stamp) {
      // grab latest transform
      tf_listener_.lookupTransform(tf_sensor_frame_, tf_odom_frame_,
                                   latest_time, tf_transform);

      kindr::minimal::QuatTransformation kindr_transform;
      tf::transformTFToKindr(tf_transform, &kindr_transform);
      motion_correction_->addTransformation(tf_transform.stamp_,
                                            kindr_transform);

      prev_tf_stamp = tf_transform.stamp_;
    }
  }

  void pointcloudCallback(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
    std::shared_ptr<pcl::PointCloud<PointOS1>> pointcloud_pcl_in =
        std::make_shared<pcl::PointCloud<PointOS1>>();
    pcl::fromROSMsg(*pointcloud_msg_in, *pointcloud_pcl_in);
    motion_correction_->addPointCloudToQueue(pointcloud_pcl_in);

    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl_out;
    while (
        motion_correction_->processNextQueuedPointcloud(&pointcloud_pcl_out)) {
      pointcloud_pub_.publish(pointcloud_pcl_out);
      pointcloud_pcl_out.clear();
    }
  }

  MotionCorrectionNode(const ros::NodeHandle& nh,
                       const ros::NodeHandle nh_private)
      : nh_(nh), nh_private_(nh_private) {
    int pointcloud_queue_length;
    nh_private_.param("pointcloud_queue_length", pointcloud_queue_length, 100);
    bool reflectivity_as_intensity;
    nh_private_.param("reflectivity_as_intensity", reflectivity_as_intensity,
                      false);
    nh_private_.param("sensor_frame", tf_sensor_frame_, std::string("os1"));
    nh_private_.param("odom_frame", tf_odom_frame_, std::string("odom"));

    tf_timer_ = nh.createTimer(ros::Duration(0.01),
                               &MotionCorrectionNode::tfTimerCallback, this);

    motion_correction_ = std::make_shared<MotionCorrection>(
        pointcloud_queue_length, reflectivity_as_intensity);

    pointcloud_sub_ = nh_.subscribe(
        "pointcloud", 10, &MotionCorrectionNode::pointcloudCallback, this);
    pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI>>(
        "corrected_pointcloud", 10);
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pointcloud_pub_;
  ros::Subscriber pointcloud_sub_;

  ros::Timer tf_timer_;
  tf::TransformListener tf_listener_;

  std::string tf_odom_frame_;
  std::string tf_sensor_frame_;
  std::shared_ptr<MotionCorrection> motion_correction_;
};

}  // namespace OS1
}  // namespace ouster_ros

int main(int argc, char** argv) {
  ros::init(argc, argv, "ouster_motion_compensation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ouster_ros::OS1::MotionCorrectionNode node(nh, nh_private);

  ros::spin();
  return 0;
}
