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
#include <chrono>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include "file_transfer_server.cpp"

int i = 0;
ros::Publisher pub_;

Server_socket S(8008);

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

double no_frame = 0, gremoval_time = 0;
void callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // Start Log Stopwatch
    auto start_time = std::chrono::high_resolution_clock::now();
    ros::Time begin = ros::Time::now();

    //Sensor msgs to pointcloud2 pointer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud1);

    pcl::io::OctreePointCloudCompression<pcl::PointXYZ> *PointCloudEncoder;

    // for a full list of profiles see: /io/include/pcl/compression/compression_profiles.h
    pcl::io::compression_Profiles_e compressionProfile = pcl::io::HIGH_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

    // instantiate point cloud compression for encoding and decoding
    PointCloudEncoder = new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>(compressionProfile, false);

    // stringstream to store compressed point cloud
    std::stringstream compressedData;

    // compress point cloud
    PointCloudEncoder->encodePointCloud(cloud1, compressedData);

    // Post-Compression time
    auto post_comp_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sw_post_comp = post_comp_time - start_time;

    //writing to  file
    std::ofstream myfile;

    std::stringstream s;
    s << i++;
    std::string txtfpath = "/home/untitlederror-09/Documents/code/Projects/LiDAR_datastream_comdecom/test_file_streams/" + s.str();
    txtfpath = txtfpath + ".txt";
    myfile.open(txtfpath.c_str(), ios::out);
    myfile << compressedData.str();
    myfile.close();
    FileMD *file = new FileMD(std::__cxx11::string(txtfpath));

    // Time to write to file
    auto post_fsave_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sw_fwrite = post_fsave_time - post_comp_time;

    // Transmit file to client
    S.transmit_file(file);

    // Time to transfer to file
    auto post_ftranfer_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> sw_ftransfer = post_ftranfer_time - post_fsave_time;
    //-----------------------------------Ground Removed---------------------------------------
    /*
    ros::Time time_at_end = ros::Time::now();
    std::cout << time_at_end << std::endl;
    std::cout << "time for ground removal " << time_at_end - begin << std::endl;
    gremoval_time += (time_at_end - begin).toSec();
    no_frame++;
    std::cout << "average ground removal time: " << (gremoval_time / no_frame) << std::endl;
    std::cout << "Total no of frames: " << no_frame << std::endl;
*/
    //------------------------------------------------------------------------------------------

    delete (PointCloudEncoder);
    delete (file);
    std::remove(txtfpath.c_str()); // delete file

    cout << i << "," << sw_post_comp.count() << "," << sw_fwrite.count() << "," << sw_ftransfer.count() << endl;
}

int main(int argc, char **argv)
{
    //Initialize Server Socket
    // S = new Server_socket(atoi(argv[3]));

    cout<<"No.,sw_post_comp,sw_fwrite,sw_ftransfer"<<endl;

    //Initiate ROS
    ros::init(argc, argv, "ground_removal_ring");
    ros::NodeHandle nh_ground_removal_new;
    ros::Subscriber sub_ = nh_ground_removal_new.subscribe("/velodyne_points", 1, callback);

    pub_ = nh_ground_removal_new.advertise<sensor_msgs::PointCloud2>("/gremoved_topic", 1);
    //pub_ = nh_ground_removal_new.advertise<PointCloud>("/published_topic", 1);

    ros::spin();

    // delete(S);

    return 0;
}