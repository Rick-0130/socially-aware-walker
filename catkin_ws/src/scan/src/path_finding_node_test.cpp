#include <chrono>
#include <signal.h>
#include <math.h>
#include <algorithm>

// ROS
#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/Float32.h>

// TF
#include <tf/transform_listener.h>
#include "tf/transform_datatypes.h"

// Custom library
#include "Astar.hpp"

using namespace std;


template<class ForwardIterator>
inline size_t argmin(ForwardIterator first, ForwardIterator last) {
    return std::distance(first, std::min_element(first, last));
}

template<class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last) {
    return std::distance(first, std::max_element(first, last));
}


class AstarPathfindingNode {
public:
    AstarPathfindingNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    static void sigint_cb(int sig);
    void localmap_cb(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr);
    void footprint_cb(const geometry_msgs::PolygonStamped::ConstPtr &footprint_msg_ptr);
    void progress_cb(const std_msgs::Float32::ConstPtr &msg_ptr);
    int get_cost(vector<int8_t> vec, int map_width, int map_height, int target_idx);
    geometry_msgs::Point generate_sub_goal(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr, tf::StampedTransform tf_base2odom);
    bool is_footprint_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr, geometry_msgs::PolygonStamped::ConstPtr &footprint_ptr);
    bool is_path_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr, nav_msgs::Path::Ptr path_ptr, tf::StampedTransform tf_base2odom);
    //bool is_robot_following_path(nav_msgs::Path::Ptr path_ptr, double tracking_progress_percentage, tf::StampedTransform tf_base2odom);

    void timer_cb(const ros::TimerEvent&);

    // ROS related
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_localmap_;
    ros::Subscriber sub_footprint_;
    ros::Subscriber sub_tracking_progress_percentage_;
    ros::Publisher pub_walkable_path_;
    ros::Publisher pub_marker_array_;
    ros::Timer timer_;
    nav_msgs::OccupancyGrid::ConstPtr localmap_ptr_;
    nav_msgs::Path::Ptr walkable_path_ptr_;
    geometry_msgs::PolygonStamped::ConstPtr footprint_ptr_;
    string path_frame_id_;

    // TF related
    tf::TransformListener tflistener_;

    // Sub-goal related
    visualization_msgs::Marker mkr_subgoal_candidate_;
    visualization_msgs::Marker mrk_subgoal_;
    double subgoal_timer_interval_;
    double solver_timeout_ms_; 
    bool flag_planning_busy_;
    
    // Feedback of path tracking module 
    double tracking_progress_percentage_ = 0;      // to check the progress of tracking module

    double time=0;
    // A* clever trick
    double path_start_offsetx_;
    double path_start_offsety_;
};


AstarPathfindingNode::AstarPathfindingNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // Signal handler
    signal(SIGINT, sigint_cb);

    // ROS parameters
    ros::param::param<double>("~solver_timeout_ms", solver_timeout_ms_, 100.0);
    ros::param::param<double>("~subgoal_timer_interval", subgoal_timer_interval_, 0.1);
    ros::param::param<double>("~path_start_offsetx", path_start_offsetx_, 0.44);    // trick: start path from robot front according to the robot footprint
    ros::param::param<double>("~path_start_offsety", path_start_offsety_, 0.0);
    // Fixed parameters
    ros::param::param<string>("~path_frame_id", path_frame_id_, "odom");

    // ROS publishers & subscribers
    sub_localmap_ = nh_.subscribe("local_map", 5, &AstarPathfindingNode::localmap_cb, this);
    sub_footprint_= nh_.subscribe("footprint", 1, &AstarPathfindingNode::footprint_cb, this);
    sub_tracking_progress_percentage_ = nh_.subscribe("tracking_progress", 1, &AstarPathfindingNode::progress_cb, this);
    pub_walkable_path_ = nh_.advertise<nav_msgs::Path>("walkable_path", 1);
    pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("path_vis", 1);

    // Marker init
    mkr_subgoal_candidate_.header.frame_id = "base_link";
    mkr_subgoal_candidate_.ns = "subgoal_candidate";
    mkr_subgoal_candidate_.type = visualization_msgs::Marker::LINE_LIST;
    mkr_subgoal_candidate_.action = visualization_msgs::Marker::ADD;
    mkr_subgoal_candidate_.pose.orientation.w = 1.0;
    mkr_subgoal_candidate_.scale.x = 0.05;
    mkr_subgoal_candidate_.color.a = 0.2; // Don't forget to set the alpha!
    mkr_subgoal_candidate_.color.r = 1.0;
    mkr_subgoal_candidate_.color.g = 1.0;
    mkr_subgoal_candidate_.color.b = 1.0;
    mkr_subgoal_candidate_.lifetime = ros::Duration(2.0);

    mrk_subgoal_.header.frame_id = path_frame_id_;
    mrk_subgoal_.ns = "subgoal";
    mrk_subgoal_.type = visualization_msgs::Marker::SPHERE;
    mrk_subgoal_.action = visualization_msgs::Marker::ADD;
    mrk_subgoal_.pose.orientation.w = 1.0;
    mrk_subgoal_.scale.x = 0.4;
    mrk_subgoal_.scale.y = 0.4;
    mrk_subgoal_.scale.z = 0.4;
    mrk_subgoal_.color.a = 0.8;
    mrk_subgoal_.color.g = 1.0;
    mrk_subgoal_.lifetime = ros::Duration(2.0);
    mrk_subgoal_.id = 0;

    // Timer init
    flag_planning_busy_ = false;
    timer_ = nh_.createTimer(ros::Duration(subgoal_timer_interval_), &AstarPathfindingNode::timer_cb, this);

    cout << ros::this_node::getName() << " is ready." << endl;
}


void AstarPathfindingNode::progress_cb(const std_msgs::Float32::ConstPtr &msg_ptr) {
    tracking_progress_percentage_ = msg_ptr->data;
}


void AstarPathfindingNode::footprint_cb(const geometry_msgs::PolygonStamped::ConstPtr &footprint_msg_ptr){
    footprint_ptr_ = footprint_msg_ptr;
}


void AstarPathfindingNode::localmap_cb(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr) {
    if(!flag_planning_busy_){
        localmap_ptr_ = map_msg_ptr;
    }
}


bool AstarPathfindingNode::is_footprint_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr, geometry_msgs::PolygonStamped::ConstPtr &footprint_ptr) {
    double map_resolution = map_msg_ptr->info.resolution;
    double map_origin_x = map_msg_ptr->info.origin.position.x;
    double map_origin_y = map_msg_ptr->info.origin.position.y;
    for(int i = 0; i < footprint_ptr->polygon.points.size(); i++){
        int map_x = std::round(footprint_ptr->polygon.points[i].x - map_origin_x) / map_resolution;
        int map_y = std::round(footprint_ptr->polygon.points[i].y - map_origin_y) / map_resolution;
        int idx = map_y * map_msg_ptr->info.width + map_x;
        if(map_msg_ptr->data[idx] >= 80 || map_msg_ptr->data[idx] < 0) {
            return false;
        }
    }
    return true;
}

// bool AstarPathfindingNode::is_robot_following_path(nav_msgs::Path::Ptr path_ptr, double tracking_progress_percentage, tf::StampedTransform tf_base2odom) {
//     if(!path_ptr || path_ptr->poses.size() < 1)
//         return false;

//     tf::Vector3 robot_position = tf_base2odom.getOrigin();
//     int target_idx = path_ptr->poses.size() * (0.99 - tracking_progress_percentage);
//     // ROS_ERROR("path lenght: %d, target_idx: %d", path_ptr->poses.size(), target_idx);
//     geometry_msgs::PoseStamped tracking_point = path_ptr->poses[target_idx];
//     // ROS_ERROR("tracking idx=%d\ttracking point=%.2f, %.2f\trobot position=%.2f,%.2f", \
//     //             target_idx, \
//     //             tracking_point.pose.position.x, \
//     //             tracking_point.pose.position.y, \
//     //             robot_position.getX(), \
//     //             robot_position.getY());
//     if(std::hypot(tracking_point.pose.position.x - robot_position.getX(), tracking_point.pose.position.y - robot_position.getY()) > 2.0)
//         return false;
//     else
//         return true;
// }


bool AstarPathfindingNode::is_path_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr, nav_msgs::Path::Ptr path_ptr, tf::StampedTransform tf_base2odom) {
    double map_resolution = map_msg_ptr->info.resolution;
    double map_origin_x = map_msg_ptr->info.origin.position.x;
    double map_origin_y = map_msg_ptr->info.origin.position.y;
    int map_width = map_msg_ptr->info.width;
    int map_height = map_msg_ptr->info.height;

    if(!path_ptr || path_ptr->poses.size() == 0){
        // ROS_WARN("Empty path, skip");
        return false;
    }

    // Transformation matrix from odom to baselink (for localmap check)
    tf::Matrix3x3 rot_odom2base = tf_base2odom.getBasis().transpose();
    tf::Vector3 tras_odom2base = rot_odom2base * tf_base2odom.getOrigin() * (-1);
    
    for(std::vector<geometry_msgs::PoseStamped>::iterator it = path_ptr->poses.begin() ; it != path_ptr->poses.end(); ++it) {
        tf::Matrix3x3 mat_raw(tf::Quaternion(it->pose.orientation.x, it->pose.orientation.y, it->pose.orientation.z, it->pose.orientation.w));
        tf::Vector3 vec_raw(it->pose.position.x, it->pose.position.y, it->pose.position.z);

        tf::Matrix3x3 mat_transformed = rot_odom2base * mat_raw;
        tf::Vector3 vec_transformed = rot_odom2base * vec_raw + tras_odom2base;

        int map_x = std::round((vec_transformed.getX() - map_origin_x) / map_resolution);
        int map_y = std::round((vec_transformed.getY() - map_origin_y) / map_resolution);
        int idx = map_y * map_width + map_x;
        if(map_msg_ptr->data[idx] >= 20 || map_msg_ptr->data[idx] < 0) {
            return false;
        }
    }

    return true;
}


geometry_msgs::Point AstarPathfindingNode::generate_sub_goal(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr, tf::StampedTransform tf_base2odom) {
    double map_resolution = map_msg_ptr->info.resolution;
    double map_origin_x = map_msg_ptr->info.origin.position.x;
    double map_origin_y = map_msg_ptr->info.origin.position.y;
    int map_width = map_msg_ptr->info.width;
    int map_height = map_msg_ptr->info.height;
    // int map_limit = map_width * map_height;

    tf::Vector3 trans_base2odom = tf_base2odom.getOrigin();
    tf::Matrix3x3 rot_base2odom = tf_base2odom.getBasis();

    // Marker reset
    visualization_msgs::MarkerArray mrk_array;
    mkr_subgoal_candidate_.header.frame_id = map_msg_ptr->header.frame_id;
    mkr_subgoal_candidate_.points.clear();  

    tf::Vector3 subgoal_vec;
    geometry_msgs::Point subgoal_pt;

    // Sub-goal candidates
    std::vector<double> candidate_score_list;
    double prefer_subgoal_distance = 4.0;
    double distance_resolution = map_resolution * 2;//0.4
    int max_distance_idx = std::round(prefer_subgoal_distance / distance_resolution);//10

    //  2021/08/19
    // for(int i=2; i <= max_distance_idx; i++)
    // {
    //     geometry_msgs::Point pt_last;
    //     pt_last.x = i*distance_resolution*std::sin(M_PI / 36 * (27))+ path_start_offsetx_;
    //     pt_last.y = i*distance_resolution*std::cos(M_PI / 36 * (27));
    //     for(int j=26; j>=9; j--)
    //     {
    //         bool crush=false;
    //         double theta_from_yaxis = M_PI / 36 * (j);
    //         double tmp_dis = distance_resolution * i;
    //         for(int k=1;k*0.1<=tmp_dis;k++)
    //         {
    //             int map_x = std::round((0.1*k * std::sin(theta_from_yaxis) - map_origin_x + path_start_offsetx_) / map_resolution);
    //             int map_y = std::round((0.1*k * std::cos(theta_from_yaxis) - map_origin_y + path_start_offsety_) / map_resolution);
    //             int idx = map_y * map_width + map_x;
    //             if(map_msg_ptr->data[idx]>50)
    //             {
    //                 crush=true;
                    
    //             }
    //         }
            
    //         int map_x = std::round((tmp_dis * std::sin(theta_from_yaxis) - map_origin_x + path_start_offsetx_) / map_resolution);
    //         int map_y = std::round((tmp_dis * std::cos(theta_from_yaxis) - map_origin_y + path_start_offsety_) / map_resolution);
    //         int idx = map_y * map_width + map_x;

    //         // Visualization
    //         geometry_msgs::Point pt;
    //         pt.x = path_start_offsetx_;
    //         pt.y = path_start_offsety_;
    //         pt.x += tmp_dis * std::sin(theta_from_yaxis);
    //         pt.y += tmp_dis * std::cos(theta_from_yaxis);
    //         if(crush==true||j==9)
    //         {
    //                 // cout<<"j"<<j<<endl;
    //                 // cout<<"pt.x"<<pt.x<<endl;
    //                 // cout<<"pt.y"<<pt.y<<endl;
    //                 // cout<<"pt_last.x"<<pt_last.x<<endl;
    //                 // cout<<"pt_last.y"<<pt_last.y<<endl;
    //                 if(sqrt(pow(pt.x - pt_last.x, 2)+pow(pt.y - pt_last.y, 2))>0.4)
    //                 {
                        
    //                     // Calculate score
    //                     double score = sqrt(pow(pt.x - pt_last.x, 2)+pow(pt.y - pt_last.y, 2));
    //                     candidate_score_list.push_back(score);

    //                     geometry_msgs::Point pt_middle;
    //                     pt_middle.x = path_start_offsetx_;
    //                     pt_middle.y = path_start_offsety_;
    //                     mkr_subgoal_candidate_.points.push_back(pt_middle);    // Origin point
    //                     mkr_subgoal_candidate_.id = j;
    //                     pt_middle.x = (pt.x + pt_last.x)/2;
    //                     pt_middle.y = (pt.y + pt_last.y)/2;
    //                     mkr_subgoal_candidate_.points.push_back(pt_middle);
    //                 }
    //                 pt_last = pt;
    //                 crush=false;
                

                
    //         }

            
    //     }

        
        
        
        
    // }

    // mkr_subgoal_candidate_.header.stamp = ros::Time();
    // mrk_array.markers.push_back(mkr_subgoal_candidate_);    

    // // Find the farthest walkable space
    // int index = argmax(candidate_score_list.begin(), candidate_score_list.end());
    // cout << "index" << index<<endl;
    // tf::Vector3 subgoal_vec;
    // geometry_msgs::Point subgoal_pt;
    // if(index==0)
    // {
    //     // Convert goal point from base_link coordinate to odom coordinate
    //     tf::Vector3 subgoal_vec(0.44, 
    //                             0, 
    //                             0);
    //     subgoal_pt.x = path_start_offsetx_;
    //     subgoal_pt.y = path_start_offsety_;
    //     cout << "No way to go,just stay!!!" << index<<endl;
    // }
    // else
    // {
    //     // Convert goal point from base_link coordinate to odom coordinate
    //     tf::Vector3 subgoal_vec(mkr_subgoal_candidate_.points[index * 2 + 1].x, 
    //                             mkr_subgoal_candidate_.points[index * 2 + 1].y, 
    //                             mkr_subgoal_candidate_.points[index * 2 + 1].z);
    //     subgoal_pt.x = mkr_subgoal_candidate_.points[index * 2 + 1].x;
    //     subgoal_pt.y = mkr_subgoal_candidate_.points[index * 2 + 1].y;
    
    
    // }
    
        for(int m=3;m<5;m++)
    {   
        
        for(int i=2; i <= max_distance_idx; i++)
        {
            geometry_msgs::Point pt_last;
            pt_last.x = i*distance_resolution*std::sin(M_PI / 36 * (27))+ path_start_offsetx_;
            pt_last.y = i*distance_resolution*std::cos(M_PI / 36 * (27));
            cout<<"pt_last"<<pt_last<<endl;
            for(int j=27; j>=9; j--)
            {
                bool crush=false;
                double theta_from_yaxis = M_PI / 36 * (j);
                double tmp_dis = distance_resolution * i;
                for(int k=1;k*0.1<=tmp_dis;k++)
                {
                    int map_x = std::round((0.1*k * std::sin(theta_from_yaxis) - map_origin_x + path_start_offsetx_) / map_resolution);
                    int map_y = std::round((0.1*k * std::cos(theta_from_yaxis) - map_origin_y + path_start_offsety_) / map_resolution);
                    int idx = map_y * map_width + map_x;
                    if(map_msg_ptr->data[idx]>20*m)
                    {
                        crush=true;
                        
                    }
                }
                
                int map_x = std::round((tmp_dis * std::sin(theta_from_yaxis) - map_origin_x + path_start_offsetx_) / map_resolution);
                int map_y = std::round((tmp_dis * std::cos(theta_from_yaxis) - map_origin_y + path_start_offsety_) / map_resolution);
                int idx = map_y * map_width + map_x;

                // Visualization
                geometry_msgs::Point pt;
                pt.x = path_start_offsetx_;
                pt.y = path_start_offsety_;
                pt.x += tmp_dis * std::sin(theta_from_yaxis);
                pt.y += tmp_dis * std::cos(theta_from_yaxis);
                if(crush==true||j==9)
                {
                        // cout<<"j"<<j<<endl;
                        // cout<<"pt.x"<<pt.x<<endl;
                        // cout<<"pt.y"<<pt.y<<endl;
                        // cout<<"pt_last.x"<<pt_last.x<<endl;
                        // cout<<"pt_last.y"<<pt_last.y<<endl;
                        if(sqrt(pow(pt.x - pt_last.x, 2)+pow(pt.y - pt_last.y, 2))>0.6)
                        {
                            
                            // Calculate score
                            double score = sqrt(pow(pt.x - pt_last.x, 2)+pow(pt.y - pt_last.y, 2));
                            candidate_score_list.push_back(score);

                            geometry_msgs::Point pt_middle;
                            pt_middle.x = path_start_offsetx_;
                            pt_middle.y = path_start_offsety_;
                            mkr_subgoal_candidate_.points.push_back(pt_middle);    // Origin point
                            mkr_subgoal_candidate_.id = j;
                            pt_middle.x = (pt.x + pt_last.x)/2;
                            pt_middle.y = (pt.y + pt_last.y)/2;
                            mkr_subgoal_candidate_.points.push_back(pt_middle);
                        }
                        pt_last = pt;
                        crush=false;
                    

                    
                }

                
            }

            
            
            
            
        }
        mkr_subgoal_candidate_.header.stamp = ros::Time();
        mrk_array.markers.push_back(mkr_subgoal_candidate_);    

        // Find the farthest walkable space
        int index = argmax(candidate_score_list.begin(), candidate_score_list.end());
        //cout << "index" << index<<endl;
        
        
        if(index==0 && m==4)
        {
            // Convert goal point from base_link coordinate to odom coordinate
            tf::Vector3 subgoal_vec(0.44, 
                                    0, 
                                    0);
            subgoal_pt.x = path_start_offsetx_;
            subgoal_pt.y = path_start_offsety_;
            cout << "No way to go,just stay!!!" << index<<endl;
        }
        else if(index!=0)
        {
            // Convert goal point from base_link coordinate to odom coordinate
            tf::Vector3 subgoal_vec(mkr_subgoal_candidate_.points[index * 2 + 1].x, 
                                    mkr_subgoal_candidate_.points[index * 2 + 1].y, 
                                    mkr_subgoal_candidate_.points[index * 2 + 1].z);
            subgoal_pt.x = mkr_subgoal_candidate_.points[index * 2 + 1].x;
            subgoal_pt.y = mkr_subgoal_candidate_.points[index * 2 + 1].y;
        
            m=5;
        }
    }


    subgoal_vec = rot_base2odom * subgoal_vec + trans_base2odom;
    mrk_subgoal_.pose.position.x = subgoal_vec.getX();
    mrk_subgoal_.pose.position.y = subgoal_vec.getY();
    mrk_subgoal_.header.stamp = ros::Time();
    mrk_array.markers.push_back(mrk_subgoal_);
    // Publish marker array
    pub_marker_array_.publish(mrk_array);

    
    
    cout << "subgoal_pt.x" << subgoal_pt.x<<endl;
    cout << "subgoal_pt.y" << subgoal_pt.y<<endl;
    return subgoal_pt;
}


int AstarPathfindingNode::get_cost(vector<int8_t> vec, int map_width, int map_height, int target_idx) {
    int kernel_size = 5;
    int bound = kernel_size / 2;
    int cost = 0;
    for(int y = -bound; y <= bound; y++) {
        for (int x = -bound; x <= bound; x++) {
            int op_idx = target_idx + x + map_width * y;
            if(op_idx < 0 || op_idx > map_width * map_height) continue;           // upper and bottom bound
            else if(abs((op_idx % map_width) - (target_idx % map_width)) > bound) continue;  // left and right bound
            else 
                cost += vec[op_idx];
        }
    }
    return cost / (kernel_size * kernel_size);
}


void AstarPathfindingNode::timer_cb(const ros::TimerEvent&){
    // Lock
    flag_planning_busy_ = true;
    time+=0.1;
    if(!localmap_ptr_){
        ROS_WARN("Empty local map, skip");
    } else{
        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        double map_resolution = localmap_ptr_->info.resolution;
        double map_origin_x = localmap_ptr_->info.origin.position.x;
        double map_origin_y = localmap_ptr_->info.origin.position.y;
        int map_width = localmap_ptr_->info.width;
        int map_height = localmap_ptr_->info.height;
    
        // Get transformation from base to odom
        tf::StampedTransform tf_base2odom;
        try{
            tflistener_.waitForTransform(path_frame_id_, "/base_link",
                                            ros::Time(0), ros::Duration(subgoal_timer_interval_));
            tflistener_.lookupTransform(path_frame_id_, "/base_link",
                                            ros::Time(0), tf_base2odom);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        tf::Vector3 trans_base2odom = tf_base2odom.getOrigin();
        tf::Matrix3x3 rot_base2odom = tf_base2odom.getBasis();

        if(!is_footprint_safe(localmap_ptr_, footprint_ptr_)) {
            ROS_ERROR("Collision detected!!");
            walkable_path_ptr_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
            walkable_path_ptr_->header.stamp = ros::Time();
            pub_walkable_path_.publish(walkable_path_ptr_);
            flag_planning_busy_ = false;
            return;
        }
        else if((is_path_safe(localmap_ptr_, walkable_path_ptr_, tf_base2odom))&&(time<2)&&(tracking_progress_percentage_<0.9))
        {
            
            walkable_path_ptr_->header.stamp = ros::Time();
            pub_walkable_path_.publish(walkable_path_ptr_);
            flag_planning_busy_ = false;
            return;
        }
        else 
        {
            time=0;
            cout <<"tracking_progress_percentage_"<<tracking_progress_percentage_<<endl;
            geometry_msgs::Point subgoal_pt;
            if((1.0 - tracking_progress_percentage_) < 0.1){
                // Arrival situation
                subgoal_pt = generate_sub_goal(localmap_ptr_, tf_base2odom);
                ROS_WARN("Almost arrivied, generate new goal: (%.2f, %.2f)", subgoal_pt.x, subgoal_pt.y);
            }else if(walkable_path_ptr_){
                // Unsafe path situation
                subgoal_pt = generate_sub_goal(localmap_ptr_, tf_base2odom);
                ROS_WARN("Old path is not safe, generate new goal: (%.2f, %.2f)", subgoal_pt.x, subgoal_pt.y);
            }
            else {
                // New plan situation
                subgoal_pt = generate_sub_goal(localmap_ptr_, tf_base2odom);
                ROS_WARN("No any old path,    start plan...");
            }

            


            if(std::hypot(subgoal_pt.x - path_start_offsetx_, subgoal_pt.y - path_start_offsety_) < 0.2){
                walkable_path_ptr_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
                walkable_path_ptr_->header.stamp = ros::Time();
                pub_walkable_path_.publish(walkable_path_ptr_);
                flag_planning_busy_ = false;
                return;
            }
        
        
            //subgoal_pt = generate_sub_goal(localmap_ptr_, tf_base2odom);
        //&& \is_robot_following_path(walkable_path_ptr_, tracking_progress_percentage_, tf_base2odom)
        // else if((1.0 - tracking_progress_percentage_) > 1e-3 && \
        //         is_path_safe(localmap_ptr_, walkable_path_ptr_, tf_base2odom) ){
        //     // no need plan, just publish old path
        //     walkable_path_ptr_->header.stamp = ros::Time();
        //     pub_walkable_path_.publish(walkable_path_ptr_);
        //     flag_planning_busy_ = false;
        //     return;
        // }else{
        //     geometry_msgs::Point subgoal_pt;
        //     if((1.0 - tracking_progress_percentage_) < 1e-3){
        //         // Arrival situation
        //         subgoal_pt = generate_sub_goal(localmap_ptr_, tf_base2odom);
        //         ROS_WARN("Almost arrivied, generate new goal: (%.2f, %.2f)", subgoal_pt.x, subgoal_pt.y);
        //     }else if(walkable_path_ptr_){
        //         // Unsafe path situation
        //         subgoal_pt = generate_sub_goal(localmap_ptr_, tf_base2odom);
        //         ROS_WARN("Old path is not safe, generate new goal: (%.2f, %.2f)", subgoal_pt.x, subgoal_pt.y);
        //     }else{
        //         // New plan situation
        //         subgoal_pt = generate_sub_goal(localmap_ptr_, tf_base2odom);
        //         ROS_WARN("No any old path,    start plan...");
        //     }

        //     if(std::hypot(subgoal_pt.x - path_start_offsetx_, subgoal_pt.y - path_start_offsety_) < 0.2){
        //         walkable_path_ptr_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
        //         walkable_path_ptr_->header.stamp = ros::Time();
        //         pub_walkable_path_.publish(walkable_path_ptr_);
        //         flag_planning_busy_ = false;
        //         return;
        //     }

            // A* path planning
            walkable_path_ptr_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
            walkable_path_ptr_->header.frame_id = path_frame_id_;
            Astar::Solver solver;
            
            
            // subgoal_pt.x = 3;
            // subgoal_pt.y = 0;
            // coordinate to map grid
            // Trick: start plan from the grid which is in front of robot
            int origin_idx = std::round((-map_origin_y + path_start_offsety_) / map_resolution) * map_width + 
                                std::round((-map_origin_x + path_start_offsetx_) / map_resolution);
            int map_x = std::round((subgoal_pt.x - map_origin_x) / map_resolution);
            int map_y = std::round((subgoal_pt.y - map_origin_y) / map_resolution);
            int target_idx = map_y * map_width + map_x;

            bool flag_success = solver.solve_ros(localmap_ptr_, walkable_path_ptr_, origin_idx, target_idx, solver_timeout_ms_);
            if(flag_success){
                // Convert path from base_link coordinate to odom coordinate
                for(std::vector<geometry_msgs::PoseStamped>::iterator it = walkable_path_ptr_->poses.begin() ; it != walkable_path_ptr_->poses.end(); ++it) {
                    tf::Matrix3x3 mat_raw(tf::Quaternion(it->pose.orientation.x, it->pose.orientation.y, it->pose.orientation.z, it->pose.orientation.w));
                    tf::Vector3 vec_raw(it->pose.position.x, it->pose.position.y, it->pose.position.z);
                    
                    tf::Matrix3x3 mat_transformed = rot_base2odom * mat_raw;
                    tf::Vector3 vec_transformed = rot_base2odom * vec_raw + trans_base2odom;
                    it->pose.position.x = vec_transformed.getX();
                    it->pose.position.y = vec_transformed.getY();
                    it->pose.position.z = vec_transformed.getZ();
                    // tf::Quaternion q_transformed;
                    // mat_transformed.getRotation(q_transformed);
                    // tf::quaternionTFToMsg(q_transformed, it->pose.orientation);
                }
                walkable_path_ptr_->header.stamp = ros::Time();
                pub_walkable_path_.publish(walkable_path_ptr_);
            }
            else{
                // Publish empty path if there are no path finding solution.
                ROS_WARN("No solution for path finding in timeout: %.1f ms", solver_timeout_ms_);
                walkable_path_ptr_->header.stamp = ros::Time();
                pub_walkable_path_.publish(walkable_path_ptr_);
            }
        
        }        
        // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    }

    flag_planning_busy_ = false;
}


void AstarPathfindingNode::sigint_cb(int sig) {
    cout << "\nNode name: " << ros::this_node::getName() << " is shutdown." << endl;
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "astar_path_finding_node");
    ros::NodeHandle nh, pnh("~");
    AstarPathfindingNode node(nh, pnh);    
    ros::spin();
    return 0;
}
