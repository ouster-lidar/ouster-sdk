#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/point_cloud.h"
ros::Publisher pub;
ros::Publisher pointcloudXYZ;
ros::Publisher pointcloud2_publisher;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
void  cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	 // Create a container for the data.
 	sensor_msgs::PointCloud2 output;
 	sensor_msgs::PointCloud2 pcl_to_ros_pointcloud2;

	// Do data processing here...
	output = *input;	
	//printf ("Fields: %s %s %s %s  \n", output.fields[0].name.c_str(), output.fields[1].name.c_str(), output.fields[2].name.c_str(), output.fields[3].name.c_str());
	
	//output.fields[3].name = "intensity";
	pcl::PointCloud<pcl::PointXYZI> temp_cloud;
    pcl::fromROSMsg(output,temp_cloud);
	
	//printf ("Fields: %s %s %s %s  \n", output.fields[0].name.c_str(), output.fields[1].name.c_str(), output.fields[2].name.c_str(), output.fields[3].name.c_str());
	//std::cout << "here" << output.fields[3].name ;
	/*
       void fromROSMsg(const sensor_msgs::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
       {
            pcl::PCLPointCloud2 pcl_pc2;
            pcl_conversions::toPCL(cloud, pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
       }
       */

    //pcl::toROSMsg(cloud, pcl_to_ros_pointcloud2);//convert back to PointCloud2
	
	//publish to topics
    //pub.publish (output);
	//pointcloudXYZ.publish(cloud);
	//pointcloud2_publisher.publish(pcl_to_ros_pointcloud2);
	//ROS_INFO("Success output");//cout
}   
int   main (int argc, char** argv)
{
     // Initialize ROS
     ros::init (argc, argv, "my_pcl_tutorial");
     ros::NodeHandle nh;   
     // Create a ROS subscriber for the input point cloud
     ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/os1_node/points", 10, cloud_cb);

	 
     // Spin
     ros::spin ();
}


	