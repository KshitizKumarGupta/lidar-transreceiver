#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <string>
#include <pcl/registration/icp.h>
#include <math.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <boost/lexical_cast.hpp>
#include <algorithm>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <octomap/octomap.h>
#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/PointIndices.h>
#include <ctime>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <chrono>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include "base64.cpp"

ros::Publisher pub_2;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

double no_frame = 0, gremoval_time = 0;
void callback(const std_msgs::String &msg)
{
  std::stringstream buffer;
  buffer<<base64_decode(msg.data);

  //decoder
  pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *PointCloudDecoder;
  // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
  pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

  // instantiate point cloud compression for encoding and decoding

  PointCloudDecoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();

  // output pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>);

  // decompress point cloud
  PointCloudDecoder->decodePointCloud(buffer, cloudOut);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(*cloudOut, pcl_pc2);

  sensor_msgs::PointCloud2 rosCloud;
  pcl_conversions::fromPCL(pcl_pc2, rosCloud);

  // rosCloud.header.stamp = msg->header.stamp;
  rosCloud.header.frame_id = "velodyne";
  rosCloud.header.stamp = ros::Time::now();

  //  rosCloud.header.frame_id="velodyne";
  pub_2.publish(rosCloud);
  // cout<<"published"<<endl;

  delete (PointCloudDecoder);
}

int main(int argc, char **argv)
{
  //Initialize Server Socket
  // S = new Server_socket(atoi(argv[3]));

  // cout << "No.,sw_fwrite,sw_ftransfer" << endl;

  //Initiate ROS
  ros::init(argc, argv, "new_nh");
  ros::NodeHandle nh;
  ros::Subscriber sub_ = nh.subscribe("/b2n_data", 1, callback);

  pub_2 = nh.advertise<sensor_msgs::PointCloud2>("/n2v_data", 1);
  //pub_2 = nh.advertise<PointCloud>("/published_topic", 1);

  ros::spin();

  // delete(S);

  return 0;
}