#include <iostream>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>



using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


ros::Publisher pub_combined_image;



void scan_callback(const sensor_msgs::PointCloud2 cloud_msg){
    // cout << "hi" << endl;
    sensor_msgs::PointCloud2 output;
    output.header.stamp=ros::Time::now();
    // // Laserscan -> ROS PointCloud2
    // sensor_msgs::PointCloud2 cloud_msg;
    // projector.projectLaser(laser_msg, cloud_msg);
    // //pub_combined_image.publish(cloud_msg);


    // ROS PointCloud2 -> PCL Pointcloud
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    // for(int i = 0; i < cloud_filtered->points.size(); i++) {
       
    //     cloud_filtered->points[i].y=0;
        
    // }
    pcl::CropBox<pcl::PointXYZ> box_filter_; 
    box_filter_.setMax(Eigen::Vector4f(4, 0.5, 10.0, 1.0));
    box_filter_.setMin(Eigen::Vector4f(-4, -0.9, 0, 1.0));
    box_filter_.setKeepOrganized(false);
    box_filter_.setNegative(false);
    box_filter_.setInputCloud(cloud_raw);
    box_filter_.filter(*cloud_raw);
    //cout << cloud_raw->points.size()<< endl;
    //cout <<"first"<< ros::Time::now()-output.header.stamp<<endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    PointCloudXYZPtr cloud_filtered(new PointCloudXYZ);
    sor.setInputCloud (cloud_raw);
    sor.setLeafSize (0.07f, 0.07f, 0.07f);
    sor.filter (*cloud_filtered);
    //cout << cloud_filtered->points.size()<< endl;
    //cout <<"second"<< ros::Time::now()-output.header.stamp<<endl;

    // Remove outlier
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    PointCloudXYZPtr cloud_out(new PointCloudXYZ);
    outrem.setInputCloud(cloud_filtered);
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius(15);
    outrem.filter(*(cloud_out));
    //cout <<"third"<< ros::Time::now()-output.header.stamp<<endl;

    // PointCloudXYZPtr cloud_filtered2(new PointCloudXYZ);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> rorfilter;
    // rorfilter.setInputCloud(cloud_filtered);
    // rorfilter.setRadiusSearch(0.5);
    // rorfilter.setMinNeighborsInRadius (3);
    // rorfilter.filter (*cloud_filtered2);
    
    output.header.stamp= ros::Time::now();
    pcl::toROSMsg(*cloud_out, output);
    pub_combined_image.publish(output);
    

    // PCL Pointcloud -> ROS PointCloud2
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(*cloud_raw, output);
    // pub_combined_image.publish(output);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_node");
    ros::NodeHandle nh;
    
    // ROS related
    pub_combined_image = nh.advertise<sensor_msgs::PointCloud2>("scan_pointcloud_filtered", 1);
    ros::Subscriber scan_sub = nh.subscribe("/camera1/depth/color/points", 1, scan_callback);

    ros::spin();
    return 0;
}
