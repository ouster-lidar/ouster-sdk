
class MotionCorrectionNode {
 public:

  void tfCallback(const tf::tfMessageConstPtr &msg){
  	 
  	 tf::tfMessage
  }

  void pointcloudCallback(
      const sensor_msgs::PointCloud2::Ptr& pointcloud_msg_in) {
    pcl::PointCloud<PointOS1> pointcloud_pcl_in;
    pcl::fromROSMsg(*pointcloud_msg_in, pointcloud_pcl_in);
    MotionCorrection.addPointCloudToQueue(pointcloud_pcl_in);

    pcl::PointCloud<pcl::PointXYZI> pointcloud_pcl_out;
    while (processNextQueuedPointcloud(&pointcloud_pcl_out)) {
      pointcloud_pub.publish()
    }
  }

  MotionCorrectionNode(const ros::NodeHandle& nh,
                       const ros::NodeHandle nh_private)
      : nh_(nh), nh_private_(nh_private) {
    bool reflectance_as_intensity;
    nh_private_.getParam("reflectance_as_intensity", reflectance_as_intensity, false);
    nh_private_.getParam("tf_base_frame",tf_base_frame, "odom");

    motion_correction_ =
        std::make_shared<MotionCorrection>(reflectance_as_intensity);

    pointcloud_sub_ = nh.subscribe(
        "pointcloud", 10, &MotionCorrectionNode::pointcloudCallback, this);
    pointcloud_sub_ = nh.subscribe("/tf", 100, &MotionCorrectionNode::tfCallback, this);
    pointcloud_pub = nh_private.advertise<pcl::PointCloud<pcl::PointXYZI>>(
        "corrected_pointcloud", 10);
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pointcloud_pub_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber tf_sub_;

  std::string tf_base_frame_;
  std::string tf_child_frame_;
  std::make_shared<MotionCorrection> motion_correction_;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ouster_motion_compensation");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  MotionCorrectionNode node(nh, nh_private);

  ros::spin();
  return 0;
}