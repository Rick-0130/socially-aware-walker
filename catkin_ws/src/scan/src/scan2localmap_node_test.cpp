#include <chrono>
#include <cmath>
#include <math.h>
#include <signal.h>

#include "ros/ros.h"
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <walker_msgs/Trk3DArray.h>
#include <walker_msgs/Trk3D.h>

// TF
#include <tf/transform_listener.h>

// PCL
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h> // ros2pcl
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;


class Scan2LocalmapNode {
public:
    Scan2LocalmapNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void butterworth_filter(vector<int8_t> &vec, int map_width, int map_height, int target_idx, int peak_value);
    void asymmetric_gaussian_filter(vector<int8_t> &vec, double map_resolution, int map_width, int map_height, int target_idx, double target_yaw, double target_speed, int peak_value);
    void butterworth_filter_generate(double filter_radius, int filter_order, double map_resolution, int peak_value);
    void scan_cb(const sensor_msgs::PointCloud2 cloud_msg);
    void trk_cb(const walker_msgs::Trk3DArray::ConstPtr &msg_ptr);

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_trk_;
    ros::Publisher pub_map_;
    ros::Publisher pub_footprint_;
    string localmap_frameid_;                               // Localmap frame_id
    nav_msgs::OccupancyGrid::Ptr localmap_ptr_;             // Localmap msg
    geometry_msgs::PolygonStamped::Ptr footprint_ptr_;      // Robot footprint

    // TF listener
    tf::TransformListener* tflistener_ptr_;
    tf::StampedTransform tf_base2odom_;    

    // Inflation filter kernel
    vector<vector<int8_t> > inflation_kernel_;

    // PCL Cropbox filter
    // pcl::CropBox<pcl::PointXYZ> box_filter_; 
};


Scan2LocalmapNode::Scan2LocalmapNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // Signal handler
    signal(SIGINT, Scan2LocalmapNode::sigint_cb);

    // ROS parameters
    double inflation_radius;
    double map_resolution;
    double localmap_range;
    ros::param::param<double>("~inflation_radius", inflation_radius, 0.2);
    ros::param::param<double>("~map_resolution", map_resolution, 0.1);
    ros::param::param<double>("~localmap_range", localmap_range, 16.0);
    ros::param::param<string>("~localmap_frameid", localmap_frameid_, "base_link");

    // ROS publishers & subscribers
    //sub_scan_ = nh_.subscribe("/scan_pointcloud_filtered", 1, &Scan2LocalmapNode::scan_cb, this);
    sub_scan_ = nh_.subscribe("trk3d_result", 1,&Scan2LocalmapNode::trk_cb, this);
    pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    pub_footprint_ = nh_.advertise<geometry_msgs::PolygonStamped>("footprint", 1);


    // Prepare the transformation matrix from camera to base
    tflistener_ptr_ = new tf::TransformListener();
    ROS_INFO("Wait for TF from camera to %s in 10 seconds...", localmap_frameid_.c_str());
    try{
        tflistener_ptr_->waitForTransform(localmap_frameid_, "odom",
                                    ros::Time(), ros::Duration(10.0));
        tflistener_ptr_->lookupTransform(localmap_frameid_, "odom",
                                    ros::Time(), tf_base2odom_);
        ROS_INFO("Done.");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from camera to %s: %s. Aborting...", localmap_frameid_.c_str(), ex.what());
        exit(-1);
    }


    // Initialize localmap meta information
    localmap_ptr_ = nav_msgs::OccupancyGrid::Ptr(new nav_msgs::OccupancyGrid());
    localmap_ptr_->info.width = localmap_range * 2 / map_resolution;
    localmap_ptr_->info.height = localmap_range  / map_resolution;
    localmap_ptr_->info.resolution = map_resolution;
    localmap_ptr_->info.origin.position.x = -localmap_ptr_->info.resolution * localmap_ptr_->info.width / 2;
    localmap_ptr_->info.origin.position.y = -localmap_ptr_->info.resolution * localmap_ptr_->info.height / 2;
    localmap_ptr_->info.origin.orientation.w = 1.0;
    localmap_ptr_->data.resize(localmap_ptr_->info.width * localmap_ptr_->info.height);
    localmap_ptr_->header.frame_id = localmap_frameid_;
    ROS_INFO("Default range of localmap:+-%.1f m, size:%dx%d", 
                localmap_range, localmap_ptr_->info.width, localmap_ptr_->info.height);
    

    //walker's footprint
    // Footprint generator
    footprint_ptr_ = geometry_msgs::PolygonStamped::Ptr(new geometry_msgs::PolygonStamped());
    footprint_ptr_->header.frame_id = localmap_frameid_;
    geometry_msgs::Point32 pt;
    pt.x = -0.1, pt.y = 0.3314, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.3, pt.y = 0.3314, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.4414, pt.y = 0.19, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.4414, pt.y = -0.19, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.3, pt.y = -0.3314, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = -0.1, pt.y = -0.3314, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = -0.1, pt.y = -0.2014, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.25, pt.y = -0.2014, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.3561, pt.y = -0.0953, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.3561, pt.y = 0.0953, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = 0.25, pt.y = 0.2014, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);
    pt.x = -0.1, pt.y = 0.2014, pt.z = 0.0;
    footprint_ptr_->polygon.points.push_back(pt);


    // Filter kernel generator
    butterworth_filter_generate(inflation_radius, 6, map_resolution, 100);
}

void Scan2LocalmapNode::asymmetric_gaussian_filter(vector<int8_t> &vec, double map_resolution, int map_width, int map_height, int target_idx, double target_yaw, double target_speed, int peak_value) {


    if(peak_value>80)
        peak_value=80;
    // Gaussian Filter kernel
    vector<vector<int8_t> > agf_kernel;
    // double kernel_range = 2.4 * 2;
    double kernel_range = 7.5 * 2;
    double max_kernel_range = kernel_range / 2 * 1.0000001;
    for(double y = -kernel_range / 2 ; y <= max_kernel_range; y += map_resolution){
        vector<int8_t> tmp_row;
        for(double x = -kernel_range / 2; x <= max_kernel_range; x += map_resolution){
            double sigma_head = 1;
            // double sigma_head = 0.5;
            double sigma_side = sigma_head * 2 / 5;
            double sigma_rear = sigma_head / 2;

            double alpha = atan2(-y, -x) - target_yaw - M_PI * 0.5;
            double alpha_prime = atan2(sin(alpha), cos(alpha));
            double sigma_front = (alpha_prime > 0)? sigma_head : sigma_rear;
            double sin_p2 = pow(sin(target_yaw), 2);
            double cos_p2 = pow(cos(target_yaw), 2);
            double sigma_side_p2 = pow(sigma_side, 2);
            double sigma_front_p2 = pow(sigma_front, 2);
            double g_a = cos_p2 / (2 * sigma_front_p2) + sin_p2 / (2 * sigma_side_p2);
            double g_b = sin(2 * target_yaw) / (4 * sigma_front_p2) - sin(2 * target_yaw) / (4 * sigma_side_p2);
            double g_c = sin_p2 / (2 * sigma_front_p2) + cos_p2 / (2 * sigma_side_p2);
            double z = 1.0 / exp(g_a * pow(x, 2) + 2 * g_b * x * y + g_c * pow(y, 2)) * peak_value;
            tmp_row.push_back(z);
        }
        agf_kernel.push_back(tmp_row);
    }


    agf_kernel[agf_kernel.size() / 2][agf_kernel[0].size() / 2] = peak_value;

    
    int kernel_size = agf_kernel.size();
    int bound = agf_kernel.size() / 2;

    int min_bound = -bound;
    int max_bound = (bound + kernel_size % 2);
    int max_map_idx = map_width * map_height;
    for(int y = min_bound; y < max_bound; y++) {
        for (int x = min_bound; x < max_bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            int8_t op_kernel_val = agf_kernel[y + bound][x + bound];

            // if(vec[op_idx] < 0) continue;                                 // do not apply filter out of laser range
            if(op_kernel_val == 0 || vec[op_idx] >= op_kernel_val) continue;
            else if((target_idx % map_width + x) < map_width/2 ) continue; 
            else if(op_idx < 0 || op_idx > max_map_idx) continue;          // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) > bound) continue;  // left and right bound
            else{
                //int tmp_val = op_kernel_val + vec[op_idx];
                vec[op_idx] = (op_kernel_val >= 100)? 100 : op_kernel_val;
            }
        }
    }


}


void Scan2LocalmapNode::butterworth_filter(vector<int8_t> &vec, int map_width, int map_height, int target_idx, int peak_value) {
    int kernel_size = inflation_kernel_.size();
    int bound = inflation_kernel_.size() / 2;
    for(int y = -bound; y <= bound; y++) {
        for (int x = -bound; x <= bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            // if(vec[op_idx] < 0) continue;                                 // do not apply filter out of laser range
            if(inflation_kernel_[y + bound][x + bound] == 0 || vec[op_idx] >= inflation_kernel_[y + bound][x + bound]) continue;
            else if(op_idx < 0 || op_idx > map_width * map_height) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) > bound) continue;  // left and right bound
            else 
                vec[op_idx] = inflation_kernel_[y + bound][x + bound];
        }
    }
}

//generate a filter kernel with  butterworth_filter
void Scan2LocalmapNode::butterworth_filter_generate(double filter_radius, int filter_order, double map_resolution, int peak_value) {
    double kernel_range = filter_radius * 4;
    cout << "Filter kernel:" << endl;
    for(double y = -kernel_range / 2 ; y <= kernel_range / 2 * 1.00000001; y += map_resolution){
        vector<int8_t> tmp_row;
        for(double x = -kernel_range / 2; x <= kernel_range / 2 * 1.00000001; x += map_resolution){
            double r = sqrt(x * x + y * y);
            double z = (1 / sqrt(1 + pow(r / filter_radius, (2 * filter_order)))) * peak_value;
            tmp_row.push_back(z);
        }
        inflation_kernel_.push_back(tmp_row);
    }

    for(int i = 0; i < inflation_kernel_.size(); i++){
        for(int j = 0; j < inflation_kernel_[i].size(); j++){
            if(i == inflation_kernel_.size() / 2 && j == inflation_kernel_[0].size() / 2)
                inflation_kernel_[i][j] = peak_value;
            printf("%3d,", inflation_kernel_[i][j]);
        }
        cout << endl;
    }

    cout << "Inflation kernel size: (" << inflation_kernel_.size() << ", " << inflation_kernel_[0].size() << ")" << endl;
    //check if the kernel size is odd
    if(inflation_kernel_.size() % 2 == 0){
        ROS_ERROR("Even kernel size! Please assign the new filter radius so that it can generate odd kernel size");
        exit(-1);
    }
}
/*
//convert laserscan to localmap 
void Scan2LocalmapNode::scan_cb(const sensor_msgs::PointCloud2 cloud_msg) {


    // Convert  ROS PointCloud2 --> PCL PointCloudXYZ
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_transformed(new PointCloudXYZ);
    pcl::fromROSMsg(cloud_msg, *cloud_raw);
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_transformed, tf_base2camera_);
    //apply filter
    PointCloudXYZPtr cloud_filtered(new PointCloudXYZ);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_transformed);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.5,-0.1);
    pass.setFilterLimitsNegative(false);
    pass.filter (*cloud_filtered);
    for(int i = 0; i < cloud_filtered->points.size(); i++) {
       
        cloud_filtered->points[i].y=0;
        
    }
    

    // Localmap init
    fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), 0);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height;

    for(int i = 0; i < cloud_filtered->points.size(); i++) {
        double laser_x = cloud_filtered->points[i].z;
        double laser_y = -cloud_filtered->points[i].x+ 0.3;
        //check if point is out of range
        if(fabs(laser_x) > map_height * resolution / 2)
            continue;
        else if(fabs(laser_y) > map_width * resolution / 2)
            continue;

        // Add wall(non-walkable) space
        int map_x = round((laser_x - map_origin_x) / resolution);
        int map_y = round((laser_y - map_origin_y) / resolution);
        int idx = map_y * map_width + map_x;
        
        if((0 < idx) && (idx < map_limit)){
            if(localmap_ptr_->data[idx] == 100)
                continue;
            butterworth_filter(localmap_ptr_->data, map_width, map_height, idx, 100);
        }
    }

    // Publish localmap
    ros::Time now = ros::Time(0);
    localmap_ptr_->header.stamp = now;
    pub_map_.publish(*localmap_ptr_);

    // Publish footprint
    footprint_ptr_->header.stamp = now;
    pub_footprint_.publish(*footprint_ptr_);

}
*/
void Scan2LocalmapNode::trk_cb(const walker_msgs::Trk3DArray::ConstPtr &msg_ptr) {
    //cout <<"first"<< ros::Time::now()-msg_ptr->pointcloud.header.stamp<<endl;

    sensor_msgs::PointCloud2 cloud_msg = msg_ptr->pointcloud;
    // Convert  ROS PointCloud2 --> PCL PointCloudXYZ
    PointCloudXYZPtr cloud_raw(new PointCloudXYZ);
    PointCloudXYZPtr cloud_transformed(new PointCloudXYZ);
    pcl::fromROSMsg(cloud_msg, *cloud_transformed);
    //pcl_ros::transformPointCloud(*cloud_raw, *cloud_transformed, tf_base2camera_);
    //apply filter
    PointCloudXYZPtr cloud_filtered(new PointCloudXYZ);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_transformed);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0,0.7);
    pass.setFilterLimitsNegative(false);
    pass.filter (*cloud_filtered);
    for(int i = 0; i < cloud_filtered->points.size(); i++) {
       
        cloud_filtered->points[i].y=0;
        
    }

    // Localmap init
    fill(localmap_ptr_->data.begin(), localmap_ptr_->data.end(), 0);

    double resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
    int map_limit = map_width * map_height;
    int count=0;
    for(int i = 0; i < cloud_filtered->points.size(); i++) {
        double laser_x = cloud_filtered->points[i].z+0.41;
        double laser_y = -cloud_filtered->points[i].x-0.07;
        //check if point is out of range
        if(fabs(laser_x) > map_height * resolution / 2)
            continue;
        else if(fabs(laser_y) > map_width * resolution / 2)
            continue;

        // Add wall(non-walkable) space
        int map_x = round((laser_x - map_origin_x) / resolution);
        int map_y = round((laser_y - map_origin_y) / resolution);
        int idx = map_y * map_width + map_x;
        
        if((0 < idx) && (idx < map_limit)){
            if(localmap_ptr_->data[idx] == 100)
                continue;
            butterworth_filter(localmap_ptr_->data, map_width, map_height, idx, 100);
            count++;

        }
    }
    // cout<< cloud_filtered->points.size()<<endl;
    // cout<<"count"<<count<<endl;
    //cout <<"second"<< ros::Time::now()-msg_ptr->pointcloud.header.stamp<<endl;
    try{
        tflistener_ptr_->waitForTransform(localmap_frameid_, "odom",
                                    ros::Time(0), ros::Duration(0.025));
        tflistener_ptr_->lookupTransform(localmap_frameid_, "odom",
                                    ros::Time(0), tf_base2odom_);
        //ROS_INFO("Done.");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from odom to %s: %s. Aborting...", localmap_frameid_.c_str(), ex.what());
        exit(-1);
    }
    
    // Proxemics generation
    for(int i = 0; i < msg_ptr->trks_list.size(); i++) {
        // Convert object pose from laser coordinate to base coordinate
        // tf::Vector3 pt_laser(msg_ptr->trks_list[i].x, msg_ptr->trks_list[i].y, 0);
        // tf::Vector3 pt_base = tf_base2camera_.getBasis() * pt_laser + tf_base2camera_.getOrigin();
        // tf::Quaternion q;
        // q.setRPY(0, 0, msg_ptr->trks_list[i].yaw);
        // double yaw, pitch, roll;
        // tf::Matrix3x3 mat(q);
        // mat = tf_base2camera_.getBasis() * mat;
        // mat.getEulerYPR(yaw, pitch, roll);

        // double speed = hypot(msg_ptr->trks_list[i].vx, msg_ptr->trks_list[i].vy);
        // if(speed > 0.5)
        //     speed = 0.5;
        // double dangerous = 200*msg_ptr->trks_list[i].dangerous;
        // if(dangerous > 100)
        //     dangerous = 100;
        // // Calculate object position in local map
        // int map_x = round((pt_base.getX() - map_origin_x) / resolution);
        // int map_y = round((pt_base.getY() - map_origin_y) / resolution);
        // int idx = map_y * map_width + map_x;

        // if((0 < idx) && (idx < map_limit) && (speed > 0.1)){
        //     asymmetric_gaussian_filter(localmap_ptr_->data, resolution, map_width, map_height, idx, msg_ptr->trks_list[i].yaw, speed,dangerous);
        // }
        // else if((0 < idx) && (idx < map_limit) && (speed <= 0.1)){
        //     yaw = atan2(-msg_ptr->trks_list[i].y,-msg_ptr->trks_list[i].x);
        //     asymmetric_gaussian_filter(localmap_ptr_->data, resolution, map_width, map_height, idx, yaw, speed,dangerous);
        // }
        
        for(int j=0;j<20;j++)
        {
            //cout << msg_ptr->trks_list[i].x << endl;
            tf::Vector3 pt_laser(msg_ptr->trks_list[i].x+msg_ptr->trks_list[i].vx*j*0.2, msg_ptr->trks_list[i].y+msg_ptr->trks_list[i].vy*j*0.2, 0);
            // tf::Vector3 pt_base = tf_base2odom_.getBasis() * pt_laser + tf_base2odom_.getOrigin();
            tf::Vector3 pt_base = tf_base2odom_ * pt_laser;
            tf::Quaternion q;
            // q.setRPY(0, 0, msg_ptr->trks_list[i].yaw);
            double yaw, pitch, roll;
            // tf::Matrix3x3 mat(q);
            // mat = tf_base2camera_.getBasis() * mat;
            // mat.getEulerYPR(yaw, pitch, roll);

            double speed = hypot(msg_ptr->trks_list[i].vx, msg_ptr->trks_list[i].vy);
            // if(speed>2)
            //     speed=2;
            
            double dangerous;
            // if(speed>0.5)
            //     dangerous=400*speed/(1+exp(0.3*sqrt(pt_base.getX()*pt_base.getX() + pt_base.getY()*pt_base.getY())));
            // else
            //     dangerous=400*0.5/(1+exp(0.3*sqrt(pt_base.getX()*pt_base.getX() + pt_base.getY()*pt_base.getY())));

            if(j==0)
                dangerous=400/(1+exp(0.3*sqrt(pt_base.getX()*pt_base.getX() + pt_base.getY()*pt_base.getY())));

            
            // cout <<"theta" << msg_ptr->trks_list[i].yaw << endl;    
            // Calculate object position in local map
            //cout<<"dangerous"<<dangerous<<endl;
            
            int map_x = round((pt_base.getX()- map_origin_x) / resolution);
            int map_y = round((pt_base.getY()- map_origin_y) / resolution);
            int idx = map_y * map_width + map_x;

            if((0 < idx) && (idx < map_limit) && (speed > 0.15)){
                double decrease = -cos(msg_ptr->trks_list[i].yaw);
                //cout << "decrease" << decrease <<endl;
                if(decrease < 0.5)
                    decrease = 0.5;
                asymmetric_gaussian_filter(localmap_ptr_->data, resolution, map_width, map_height, idx, msg_ptr->trks_list[i].yaw, speed,dangerous*decrease*exp(-0.05*0.02*j));
            }
            else if((0 < idx) && (idx < map_limit) && (speed <= 0.15)){
                yaw = atan2(-pt_base.getY(),-pt_base.getX());
                asymmetric_gaussian_filter(localmap_ptr_->data, resolution, map_width, map_height, idx, yaw, speed,dangerous);
            }
            
        }



        
    }
    
    // Publish localmap
    ros::Time now = ros::Time(0);
    localmap_ptr_->header.stamp = now;
    pub_map_.publish(*localmap_ptr_);

    // Publish footprint
    footprint_ptr_->header.stamp = now;
    pub_footprint_.publish(*footprint_ptr_);

}


void Scan2LocalmapNode::sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_mapping_node");
    ros::NodeHandle nh, pnh("~");
    Scan2LocalmapNode node(nh, pnh);    
    ros::spin();
    return 0;
}
