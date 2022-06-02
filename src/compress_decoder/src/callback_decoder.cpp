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
#include <ctime>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include "file_transfer_client.cpp"

ros::Publisher pub_;

Client_socket C(8008, std::string("192.168.250.26"));

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;
int i = 0;

int callback(std::string txtfpath)
{
    // Start Log Stopwatch
    auto start_time = std::chrono::high_resolution_clock::now();

    std::ifstream file;

    i++;
    file.open(txtfpath.c_str(), ios::out);
    if (!file.is_open())
    {
        cout << "[ERROR] : File could not be opened!" << endl;
        return 0;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    std::remove(txtfpath.c_str()); // delete file

    // Time to read file
    auto fread_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sw_fread = fread_time - start_time;

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

    // Time to decompress
    auto dcomp_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sw_dcomp = dcomp_time - fread_time;

    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloudOut, pcl_pc2);

    sensor_msgs::PointCloud2 rosCloud;
    pcl_conversions::fromPCL(pcl_pc2, rosCloud);

    // rosCloud.header.stamp = msg->header.stamp;
    rosCloud.header.frame_id = "velodyne";
    rosCloud.header.stamp = ros::Time::now();

    //  rosCloud.header.frame_id="velodyne";
    pub_.publish(rosCloud);

    // std::cout << "vector cleared" << std::endl;

    delete (PointCloudDecoder);

    cout << i << "," << sw_fread.count() << "," << sw_dcomp.count() << endl;
    return 1;
}

int main(int argc, char **argv)
{
    //Initializing Client Socket
    // C = new Client_socket(atoi(argv[4]), argv[3]);

    cout << "No.,sw_fread,sw_dcomp" << endl;

    //Initiate ROS
    ros::init(argc, argv, "ground_removal_ring");
    ros::NodeHandle nh_ground_removal_new;
    // ros::Subscriber sub_ = nh_ground_removal_new.subscribe("/velodyne_points", 1, callback);

    while (1)
    {
        // C->receive_file();
        pub_ = nh_ground_removal_new.advertise<sensor_msgs::PointCloud2>("/live_data", 1);
        callback(C.receive_file());
    }

    //pub_ = nh_ground_removal_new.advertise<PointCloud>("/published_topic", 1);

    // ros::spin();

    // delete(C);

    return 0;
}
