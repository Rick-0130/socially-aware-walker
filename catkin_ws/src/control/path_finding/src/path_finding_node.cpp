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
#include "a_star.hpp"
#include "localmap_utils.hpp"


static const double kMaxLateralDisRobot2TrackedPt = 0.6;
static const double kThresPercentageOfArrival = 0.7; // 0.99
static const int kThresObstacleDangerCost = 80;


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
  void finalgoal_cb(const geometry_msgs::PoseStamped::ConstPtr &goal_msg_ptr);
  void progress_cb(const std_msgs::Float32::ConstPtr &msg_ptr);
  int get_local_avg_cost(nav_msgs::OccupancyGrid::ConstPtr localmap_ptr, int target_idx);
  int get_local_max_cost(nav_msgs::OccupancyGrid::ConstPtr localmap_ptr, int target_idx);
  geometry_msgs::Point generate_subgoal(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                                        const geometry_msgs::PoseStamped::ConstPtr &finalgoal_ptr,
                                        tf::StampedTransform tf_base2odom);
  geometry_msgs::Point approach_unsafe_subgoal(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                                               nav_msgs::Path::Ptr path_ptr,
                                               tf::StampedTransform tf_odom2base);
  bool is_footprint_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                         geometry_msgs::PolygonStamped::ConstPtr &footprint_ptr);
  bool is_subgoal_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                       nav_msgs::Path::Ptr path_ptr,
                       tf::StampedTransform tf_odom2base);
  bool is_path_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                    nav_msgs::Path::Ptr path_ptr,
                    tf::StampedTransform tf_odom2base);
  bool is_robot_following_path(nav_msgs::Path::Ptr path_ptr,
                               double tracking_progress_percentage,
                               tf::StampedTransform tf_odom2base);
  bool is_path_deprecated(nav_msgs::Path::Ptr path_ptr);
  void publish_robot_status_marker(std::string str_message);

  void timer_cb(const ros::TimerEvent&);

  // ROS related
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_localmap_;
  ros::Subscriber sub_footprint_;
  ros::Subscriber sub_tracking_progress_percentage_;
  ros::Subscriber sub_finalgoal_;
  ros::Publisher pub_walkable_path_;
  ros::Publisher pub_marker_array_;
  ros::Publisher pub_marker_status_;
  ros::Timer timer_;
  nav_msgs::OccupancyGrid::ConstPtr localmap_ptr_;
  nav_msgs::Path::Ptr walkable_path_ptr_;
  geometry_msgs::PolygonStamped::ConstPtr footprint_ptr_;
  std::string path_frame_id_;

  // TF related
  tf::TransformListener tflistener_;

  // Sub-goal related
  visualization_msgs::Marker mkr_subgoal_candidate_;
  visualization_msgs::Marker mrk_subgoal_;
  visualization_msgs::Marker mrk_robot_status_;
  double subgoal_timer_interval_;
  double solver_timeout_ms_; 
  bool flag_planning_busy_;
  
  // Feedback of path tracking module 
  double tracking_progress_percentage_ = 0;    // to check the progress of tracking module

  // A* clever trick
  double path_start_offsetx_;
  double path_start_offsety_;

  bool flag_infinity_traval_;

  geometry_msgs::PoseStamped::ConstPtr finalgoal_ptr_;

  astar::Solver path_solver_;
};


AstarPathfindingNode::AstarPathfindingNode(ros::NodeHandle nh,
                                           ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
  // Signal handler
  signal(SIGINT, sigint_cb);

  // ROS parameters
  ros::param::param<double>("~solver_timeout_ms", solver_timeout_ms_, 40.0);
  ros::param::param<double>("~subgoal_timer_interval", subgoal_timer_interval_, 0.5);
  ros::param::param<double>("~path_start_offsetx", path_start_offsetx_, 0.44);  // trick: start path from robot front according to the robot footprint
  ros::param::param<double>("~path_start_offsety", path_start_offsety_, 0.0);
  ros::param::param<bool>("~flag_infinity_traval", flag_infinity_traval_, false);
  // Fixed parameters
  ros::param::param<std::string>("~path_frame_id", path_frame_id_, "odom");

  // ROS publishers & subscribers
  sub_localmap_ = nh_.subscribe("local_map", 5, &AstarPathfindingNode::localmap_cb, this);
  sub_footprint_= nh_.subscribe("footprint", 1, &AstarPathfindingNode::footprint_cb, this);
  sub_tracking_progress_percentage_ = nh_.subscribe("tracking_progress", 1, &AstarPathfindingNode::progress_cb, this);
  pub_walkable_path_ = nh_.advertise<nav_msgs::Path>("walkable_path", 1);
  pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("path_vis", 1);
  pub_marker_status_ = nh_.advertise<visualization_msgs::Marker>("robot_status", 1);
  if(!flag_infinity_traval_)
    sub_finalgoal_ = nh_.subscribe("/move_base_simple/goal", 1, &AstarPathfindingNode::finalgoal_cb, this);

  // Marker init
  mkr_subgoal_candidate_.header.frame_id = "base_link";
  mkr_subgoal_candidate_.ns = "subgoal_candidate";
  mkr_subgoal_candidate_.type = visualization_msgs::Marker::LINE_LIST;
  mkr_subgoal_candidate_.action = visualization_msgs::Marker::ADD;
  mkr_subgoal_candidate_.pose.orientation.w = 1.0;
  mkr_subgoal_candidate_.scale.x = 0.05;
  mkr_subgoal_candidate_.color.a = 0.2;
  mkr_subgoal_candidate_.color.r = 1.0;
  mkr_subgoal_candidate_.color.g = 1.0;
  mkr_subgoal_candidate_.color.b = 1.0;
  mkr_subgoal_candidate_.lifetime = ros::Duration(8.0);

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
  mrk_subgoal_.lifetime = ros::Duration(8.0);
  mrk_subgoal_.id = 0;

  mrk_robot_status_.header.frame_id = "base_link";
  mrk_robot_status_.ns = "robot_status";
  mrk_robot_status_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  mrk_robot_status_.action = visualization_msgs::Marker::ADD;
  mrk_robot_status_.pose.orientation.w = 1.0;
  mrk_robot_status_.pose.position.z = 1.5;
  mrk_robot_status_.scale.z = 0.4;
  mrk_robot_status_.color.a = 1.0;
  mrk_robot_status_.color.b = 1.0;
  mrk_robot_status_.lifetime = ros::Duration(8.0);

  // Timer init
  flag_planning_busy_ = false;
  timer_ = nh_.createTimer(ros::Duration(subgoal_timer_interval_), &AstarPathfindingNode::timer_cb, this);

  path_solver_ = astar::Solver(nh_, false, kThresObstacleDangerCost, 0.6, 0.6);

  ROS_INFO_STREAM(ros::this_node::getName() << " is ready.");
}


void AstarPathfindingNode::progress_cb(const std_msgs::Float32::ConstPtr &msg_ptr) {
  tracking_progress_percentage_ = msg_ptr->data;
}


void AstarPathfindingNode::footprint_cb(const geometry_msgs::PolygonStamped::ConstPtr &footprint_msg_ptr){
  footprint_ptr_ = footprint_msg_ptr;
}


void AstarPathfindingNode::finalgoal_cb(const geometry_msgs::PoseStamped::ConstPtr &goal_msg_ptr) {
  flag_planning_busy_ = true;

  finalgoal_ptr_ = goal_msg_ptr;

  // Get transformation from base to odom
  tf::StampedTransform tf_base2odom;
  try{
    tflistener_.waitForTransform(path_frame_id_, "/base_link",
                    ros::Time(0), ros::Duration(subgoal_timer_interval_));
    tflistener_.lookupTransform(path_frame_id_, "/base_link",
                    ros::Time(0), tf_base2odom);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("tf_error %s",ex.what());
    ros::Duration(1.0).sleep();
  }

  // Check if robot footprint is safe
  if(localmap_ptr_ && is_footprint_safe(localmap_ptr_, footprint_ptr_)) {
    geometry_msgs::Point subgoal_pt = generate_subgoal(localmap_ptr_, goal_msg_ptr, tf_base2odom);
    // subgoal_pt.x = 8.0;
    // subgoal_pt.y = 4.5;
    // A* path planning
    walkable_path_ptr_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
    walkable_path_ptr_->header.frame_id = path_frame_id_;
    // base_link coordinate to map grid
    double map_resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;

    // Trick: start plan from the grid which is in front of robot
    int origin_idx = std::round((-map_origin_y + path_start_offsety_) / map_resolution) * map_width + 
              std::round((-map_origin_x + path_start_offsetx_) / map_resolution);
    int map_x = std::round((subgoal_pt.x - map_origin_x) / map_resolution);
    int map_y = std::round((subgoal_pt.y - map_origin_y) / map_resolution);
    int target_idx = map_y * map_width + map_x;

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    bool flag_success = path_solver_.FindPathByHashmap(localmap_ptr_,
                                                       walkable_path_ptr_,
                                                       origin_idx,
                                                       target_idx,
                                                       solver_timeout_ms_);
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    if(flag_success){
      // Convert path from base_link coordinate to odom coordinate
      for(auto it = walkable_path_ptr_->poses.begin() ; it != walkable_path_ptr_->poses.end(); ++it) {
        tf::Vector3 vec_raw(it->pose.position.x, it->pose.position.y, it->pose.position.z);
        tf::Vector3 vec_transformed = tf_base2odom * vec_raw;
        tf::pointTFToMsg(vec_transformed, it->pose.position);
      }
      walkable_path_ptr_->header.stamp = ros::Time::now();
      pub_walkable_path_.publish(walkable_path_ptr_);
    }
    else{
      // Publish empty path if there are no path finding solution.
      ROS_ERROR("No solution for path finding in timeout: %.1f ms", solver_timeout_ms_);
      publish_robot_status_marker("timeout for path planning to finalgoal, skip this finalgoal assignment.");
      finalgoal_ptr_.reset();
      walkable_path_ptr_->header.stamp = ros::Time::now();
      pub_walkable_path_.publish(walkable_path_ptr_);
    }
  }
  else {
    ROS_WARN("Empty localmap or unsafe footprint");
    publish_robot_status_marker("Empty localmap or unsafe footprint, skip finalgoal assignment.");
  }

  flag_planning_busy_ = false;
}


void AstarPathfindingNode::localmap_cb(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr) {
  if(!flag_planning_busy_){
    localmap_ptr_ = map_msg_ptr;
  }
}


bool AstarPathfindingNode::is_footprint_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                       geometry_msgs::PolygonStamped::ConstPtr &footprint_ptr) {
  double map_resolution = map_msg_ptr->info.resolution;
  double map_origin_x = map_msg_ptr->info.origin.position.x;
  double map_origin_y = map_msg_ptr->info.origin.position.y;

  // Connect all footprints and check if the footprint cell is located on the dangerous cost map
  std::vector<std::pair<int, int> > footprint_cells = localmap_utils::GetFootprintCells(footprint_ptr, map_msg_ptr);
  for(int i = 0; i < footprint_cells.size(); i++) {
    int idx = footprint_cells[i].second * map_msg_ptr->info.width + footprint_cells[i].first;
    if(map_msg_ptr->data[idx] >= kThresObstacleDangerCost || map_msg_ptr->data[idx] < 0)
      return false;
  }
  return true;
}


bool AstarPathfindingNode::is_robot_following_path(nav_msgs::Path::Ptr path_ptr,
                           double tracking_progress_percentage,
                           tf::StampedTransform tf_odom2base) {
  if(!path_ptr || path_ptr->poses.size() < 1)
    return false;

  int target_idx = path_ptr->poses.size() * (0.99 - tracking_progress_percentage);
  // ROS_ERROR("path lenght: %d, target_idx: %d", path_ptr->poses.size(), target_idx);
  geometry_msgs::PoseStamped tracking_point = path_ptr->poses[target_idx];
  tf::Vector3 tracking_pt_odomframe;
  tf::pointMsgToTF(tracking_point.pose.position, tracking_pt_odomframe);
  tf::Vector3 tracking_pt_baseframe = tf_odom2base * tracking_pt_odomframe;

  if(std::abs(tracking_pt_baseframe.getY()) >= kMaxLateralDisRobot2TrackedPt){
    return false;
  }else{
    return true;
  }
}


geometry_msgs::Point AstarPathfindingNode::approach_unsafe_subgoal(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                                   nav_msgs::Path::Ptr path_ptr,
                                   tf::StampedTransform tf_odom2base) {
  double map_resolution = map_msg_ptr->info.resolution;
  double map_origin_x = map_msg_ptr->info.origin.position.x;
  double map_origin_y = map_msg_ptr->info.origin.position.y;
  int map_width = map_msg_ptr->info.width;
  int map_height = map_msg_ptr->info.height;

  geometry_msgs::Point subgoal_pt;

  if(!path_ptr || path_ptr->poses.size() == 0){
    // Return (0, 0) as return value so that indicates the subgoal is not safe
    return subgoal_pt;
  }

  // Try to choose subgoal from the old path but check its safety first
  std::vector<geometry_msgs::PoseStamped>::iterator it = path_ptr->poses.begin();
  while(it != path_ptr->poses.end()) {
    tf::Vector3 vec_odom_frame(it->pose.position.x, it->pose.position.y, it->pose.position.z);
    tf::Vector3 vec_base_frame = tf_odom2base * vec_odom_frame;
    int map_x = std::round((vec_base_frame.getX() - map_origin_x) / map_resolution);
    int map_y = std::round((vec_base_frame.getY() - map_origin_y) / map_resolution);
    int idx = map_y * map_width + map_x;
    if(get_local_max_cost(map_msg_ptr, idx) >= kThresObstacleDangerCost || map_msg_ptr->data[idx] < 0){
      it++;
    } else {
      tf::pointTFToMsg(vec_base_frame, subgoal_pt);
      return subgoal_pt;
    }
  }
  // Return (0, 0) as return value so that indicates the subgoal is not safe
  return subgoal_pt;
}


bool AstarPathfindingNode::is_subgoal_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                       nav_msgs::Path::Ptr path_ptr,
                       tf::StampedTransform tf_odom2base) {
  double map_resolution = map_msg_ptr->info.resolution;
  double map_origin_x = map_msg_ptr->info.origin.position.x;
  double map_origin_y = map_msg_ptr->info.origin.position.y;
  int map_width = map_msg_ptr->info.width;
  int map_height = map_msg_ptr->info.height;

  if(!path_ptr || path_ptr->poses.size() == 0){
    // ROS_WARN("Empty path, skip");
    return false;
  }

  // tf::Matrix3x3 rot_odom2base = tf_odom2base.getBasis();
  // tf::Vector3 tras_odom2base = tf_odom2base.getOrigin();

  std::vector<geometry_msgs::PoseStamped>::iterator it = path_ptr->poses.begin();
  tf::Vector3 vec_raw(it->pose.position.x, it->pose.position.y, it->pose.position.z);
  tf::Vector3 vec_transformed = tf_odom2base * vec_raw;

  int map_x = std::round((vec_transformed.getX() - map_origin_x) / map_resolution);
  int map_y = std::round((vec_transformed.getY() - map_origin_y) / map_resolution);
  int idx = map_y * map_width + map_x;

  if(get_local_max_cost(map_msg_ptr, idx) >= kThresObstacleDangerCost || map_msg_ptr->data[idx] < 0) {
    return false;
  }else{
    return true;
  }
}


bool AstarPathfindingNode::is_path_safe(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                    nav_msgs::Path::Ptr path_ptr,
                    tf::StampedTransform tf_odom2base) {
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
  // tf::Matrix3x3 rot_odom2base = tf_odom2base.getBasis();
  tf::Vector3 tras_odom2base = tf_odom2base.getOrigin();
  
  for(std::vector<geometry_msgs::PoseStamped>::iterator it = path_ptr->poses.begin() ; it != path_ptr->poses.end(); ++it) {
    tf::Vector3 vec_raw(it->pose.position.x, it->pose.position.y, it->pose.position.z);
    tf::Vector3 vec_transformed = tf_odom2base * vec_raw;

    int map_x = std::round((vec_transformed.getX() - map_origin_x) / map_resolution);
    int map_y = std::round((vec_transformed.getY() - map_origin_y) / map_resolution);
    int idx = map_y * map_width + map_x;
    // if(map_msg_ptr->data[idx] >= kThresObstacleDangerCost || map_msg_ptr->data[idx] < 0) {
    if(get_local_max_cost(map_msg_ptr, idx) >= kThresObstacleDangerCost || map_msg_ptr->data[idx] < 0) {
      return false;
    }
  }

  return true;
}


bool AstarPathfindingNode::is_path_deprecated(nav_msgs::Path::Ptr path_ptr) {
  if(!path_ptr || path_ptr->poses.size() == 0){
    return true;
  }

  if(ros::Time::now() - path_ptr->header.stamp > ros::Duration(5.0)){
    return true;
  }else {
    return false;
  }
}


geometry_msgs::Point AstarPathfindingNode::generate_subgoal(const nav_msgs::OccupancyGrid::ConstPtr &map_msg_ptr,
                              const geometry_msgs::PoseStamped::ConstPtr &finalgoal_ptr,
                              tf::StampedTransform tf_base2odom) {
  double map_resolution = map_msg_ptr->info.resolution;
  double map_origin_x = map_msg_ptr->info.origin.position.x;
  double map_origin_y = map_msg_ptr->info.origin.position.y;
  int map_width = map_msg_ptr->info.width;
  int map_height = map_msg_ptr->info.height;

  // Get TF from odom2base
  tf::StampedTransform tf_odom2base(tf_base2odom.inverse(), tf_base2odom.stamp_, "/base_link", path_frame_id_);
  
  // Marker reset
  visualization_msgs::MarkerArray mrk_array;
  mkr_subgoal_candidate_.header.frame_id = map_msg_ptr->header.frame_id;
  mkr_subgoal_candidate_.points.clear(); 

  // Calculate the distance from base_link to finalgoal
  tf::Vector3 vec_goal_odom_frame;
  tf::pointMsgToTF(finalgoal_ptr->pose.position, vec_goal_odom_frame);
  tf::Vector3 vec_goal_base_frame = tf_odom2base * vec_goal_odom_frame;
  double dis_base2goal = std::hypot(vec_goal_base_frame.getX(), vec_goal_base_frame.getY());



  // If finalgoal is out of localmap range
  if(int((vec_goal_base_frame.getX() - map_origin_x) / map_resolution) >= map_width  ||
     int((vec_goal_base_frame.getY() - map_origin_y) / map_resolution) >= map_height) {

    // Sub-goal candidates
    std::vector<double> candidate_score_list;
    int candidate_j_list[15] = {0};
    double prefer_subgoal_distance = 8.0;
    double distance_resolution = 0.6; // map_resolution * 2;
    for(int i = 16; i >= 2; i--) {
      double theta_from_yaxis = M_PI / 18 * i;
      int max_distance_idx = std::round(prefer_subgoal_distance / distance_resolution);
      double tmp_dis;
      double obstacle_cost;
      double max_j_score = 0.0;
      for(int j = 3; j <= max_distance_idx; j++) {
        tmp_dis = distance_resolution * j;
        int map_x = std::round((tmp_dis * std::sin(theta_from_yaxis) - map_origin_x + path_start_offsetx_) / map_resolution);
        int map_y = std::round((tmp_dis * std::cos(theta_from_yaxis) - map_origin_y + path_start_offsety_) / map_resolution);
        int idx = map_y * map_width + map_x;
        obstacle_cost = get_local_max_cost(map_msg_ptr, idx);
        if(obstacle_cost > kThresObstacleDangerCost || map_msg_ptr->data[idx] == -1) {
          tmp_dis -= distance_resolution * 3;
          break;
        }
        double dis_subgoal2finalgoal = std::hypot(tmp_dis * std::sin(theta_from_yaxis) + path_start_offsetx_ - vec_goal_base_frame.getX(),
                          tmp_dis * std::cos(theta_from_yaxis) + path_start_offsety_ - vec_goal_base_frame.getY());
        // Calculate candidate score
        double score = (1.0 - obstacle_cost / 100.0) +
                (1.0 - dis_subgoal2finalgoal / dis_base2goal / 2);
        // Find the max score among the candidates in same direction
        if(score > max_j_score){
          max_j_score = score;
          candidate_j_list[16 - i] = j;
        }
      }
      tmp_dis = distance_resolution * candidate_j_list[16 - i];
      candidate_score_list.push_back(max_j_score);
  
      // Visualization
      geometry_msgs::Point pt;
      pt.x = path_start_offsetx_;
      pt.y = path_start_offsety_;
      mkr_subgoal_candidate_.points.push_back(pt);  // Origin point
      mkr_subgoal_candidate_.id = i;
      pt.x += tmp_dis * std::sin(theta_from_yaxis);
      pt.y += tmp_dis * std::cos(theta_from_yaxis);
      mkr_subgoal_candidate_.points.push_back(pt);
  
      // candidate score
      visualization_msgs::Marker mkr_candidate_score;
      mkr_candidate_score.header.frame_id = "base_link";
      mkr_candidate_score.header.stamp = ros::Time();
      mkr_candidate_score.ns = "candidate_score";
      mkr_candidate_score.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      mkr_candidate_score.action = visualization_msgs::Marker::ADD;
      mkr_candidate_score.pose.orientation.w = 1.0;
      mkr_candidate_score.pose.position.x = pt.x;
      mkr_candidate_score.pose.position.y = pt.y;
      mkr_candidate_score.pose.position.z = 0.8;
      mkr_candidate_score.id = i;
      mkr_candidate_score.scale.z = 0.2;
      mkr_candidate_score.color.a = 0.2; // Don't forget to set the alpha!
      mkr_candidate_score.color.r = 1.0;
      mkr_candidate_score.color.g = 1.0;
      mkr_candidate_score.color.b = 1.0;
      mkr_candidate_score.text = std::to_string(max_j_score);
      mkr_candidate_score.lifetime = ros::Duration(8.0);
      mrk_array.markers.push_back(mkr_candidate_score);  
  
    }
    mkr_subgoal_candidate_.header.stamp = ros::Time();
    mrk_array.markers.push_back(mkr_subgoal_candidate_);  

    // Find the farthest walkable space
    int index = argmax(candidate_score_list.begin(), candidate_score_list.end());
    geometry_msgs::Point subgoal_pt = mkr_subgoal_candidate_.points[index * 2 + 1];

    tf::Vector3 vec_base_frame;
    tf::pointMsgToTF(subgoal_pt, vec_base_frame);
    tf::Vector3 vec_odom_frame = tf_base2odom * vec_base_frame;
    tf::pointTFToMsg(vec_odom_frame, mrk_subgoal_.pose.position);

    // Publish visualization marker array
    mrk_subgoal_.header.stamp = ros::Time();
    mrk_array.markers.push_back(mrk_subgoal_);
    pub_marker_array_.publish(mrk_array);

    // ROS_WARN("New finalgoal out of the map range: (%.2f, %.2f)", vec_goal_base_frame.getX(), vec_goal_base_frame.getY());
    return subgoal_pt;
  }
  else{
    geometry_msgs::Point subgoal_pt;
    tf::pointTFToMsg(vec_goal_base_frame, subgoal_pt);
    
    mrk_subgoal_.pose.position = finalgoal_ptr->pose.position;

    // Publish visualization marker array
    mrk_subgoal_.header.stamp = ros::Time();
    mrk_array.markers.push_back(mrk_subgoal_);
    pub_marker_array_.publish(mrk_array);

    // ROS_WARN("New finalgoal in the map range: (%.2f, %.2f)", vec_goal_base_frame.getX(), vec_goal_base_frame.getY());
    return subgoal_pt;
  }
}


int AstarPathfindingNode::get_local_max_cost(nav_msgs::OccupancyGrid::ConstPtr localmap_ptr, int target_idx) {
  int map_width = localmap_ptr->info.width;
  int map_height = localmap_ptr->info.height;
  double map_resolution = localmap_ptr->info.resolution;

  int kernel_size = int(std::floor(0.6 / map_resolution));
  kernel_size = kernel_size + (kernel_size % 2 == 0);
  int bound = kernel_size / 2;
  int cost = 0;
  int max_map_idx = map_width * map_height - 1;

  for(int y = -bound; y <= bound; y++) {
    for (int x = -bound; x <= bound; x++) {
      int op_idx = target_idx + x + map_width * y;
      if(op_idx < 0 || op_idx > max_map_idx) continue;       // upper and bottom bound
      else if(abs((op_idx % map_width) - (target_idx % map_width)) > bound) continue;  // left and right bound
      
      if(localmap_ptr->data[op_idx] > cost)
        cost = localmap_ptr->data[op_idx];
    }
  }
  return cost;
}


int AstarPathfindingNode::get_local_avg_cost(nav_msgs::OccupancyGrid::ConstPtr localmap_ptr, int target_idx) {
  int map_width = localmap_ptr->info.width;
  int map_height = localmap_ptr->info.height;
  double map_resolution = localmap_ptr->info.resolution;

  int kernel_size = int(std::floor(1.0 / map_resolution));
  kernel_size = kernel_size + (kernel_size % 2 == 0);
  int bound = kernel_size / 2;
  int cost = 0;
  int max_map_idx = map_width * map_height - 1;

  for(int y = -bound; y <= bound; y++) {
    for (int x = -bound; x <= bound; x++) {
      int op_idx = target_idx + x + map_width * y;
      if(op_idx < 0 || op_idx > max_map_idx) continue;       // upper and bottom bound
      else if(abs((op_idx % map_width) - (target_idx % map_width)) > bound) continue;  // left and right bound
      else 
        cost += localmap_ptr->data[op_idx];
    }
  }
  return cost / (kernel_size * kernel_size);
}


void AstarPathfindingNode::publish_robot_status_marker(std::string str_message){
  mrk_robot_status_.text = str_message;
  mrk_robot_status_.header.stamp = ros::Time();
  pub_marker_status_.publish(mrk_robot_status_);
}


void AstarPathfindingNode::timer_cb(const ros::TimerEvent&){
  // Lock
  if (flag_planning_busy_){
    ROS_WARN("finalgoal busy");
    return;
  }
  flag_planning_busy_ = true;

  if(!localmap_ptr_){
    ROS_INFO("Empty local map, skip");
  }
  else if(!finalgoal_ptr_){
    ROS_INFO("Empty finalgoal ptr, wait for new finalgoal");
  } 
  else{
    // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    double map_resolution = localmap_ptr_->info.resolution;
    double map_origin_x = localmap_ptr_->info.origin.position.x;
    double map_origin_y = localmap_ptr_->info.origin.position.y;
    int map_width = localmap_ptr_->info.width;
    int map_height = localmap_ptr_->info.height;
  
    // Get transformation from base to odom
    tf::StampedTransform tf_base2odom;
    try{
      tflistener_.waitForTransform(path_frame_id_, "/base_link", ros::Time(0), ros::Duration(subgoal_timer_interval_));
      tflistener_.lookupTransform(path_frame_id_, "/base_link", ros::Time(0), tf_base2odom);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::StampedTransform tf_odom2base(tf_base2odom.inverse(), tf_base2odom.stamp_, "/base_link", path_frame_id_);
    tf::Vector3 trans_base2odom = tf_base2odom.getOrigin();
    tf::Matrix3x3 rot_base2odom = tf_base2odom.getBasis();

    bool flag_footprint_safe = is_footprint_safe(localmap_ptr_, footprint_ptr_);
    bool flag_subgoal_safe = is_subgoal_safe(localmap_ptr_, walkable_path_ptr_, tf_odom2base);
    bool flag_path_safe = is_path_safe(localmap_ptr_, walkable_path_ptr_, tf_odom2base);
    bool flag_path_deprecated = is_path_deprecated(walkable_path_ptr_);
    bool flag_robot_following_path = is_robot_following_path(walkable_path_ptr_, tracking_progress_percentage_, tf_odom2base);
    double dis_robot2goal = hypot(trans_base2odom.getX() - finalgoal_ptr_->pose.position.x, trans_base2odom.getY() - finalgoal_ptr_->pose.position.y);

    if(!flag_footprint_safe) {
      ROS_ERROR("Collision detected!!");
      // Just publish an empty path
      nav_msgs::Path empty_path;
      empty_path.header.stamp = ros::Time();
      empty_path.header.frame_id = path_frame_id_;
      pub_walkable_path_.publish(empty_path);

      publish_robot_status_marker("Collision detected");
      flag_planning_busy_ = false;
      return;
    }
    else if(dis_robot2goal <= map_resolution * 2){
      ROS_WARN("Almost arrived finalgoal");
      finalgoal_ptr_.reset();
      
      nav_msgs::Path empty_path;
      empty_path.header.stamp = ros::Time();
      empty_path.header.frame_id = path_frame_id_;
      pub_walkable_path_.publish(empty_path);
      walkable_path_ptr_.reset();

      publish_robot_status_marker("finalgoal arrival");
      flag_planning_busy_ = false;
      return;
    }
    else if((dis_robot2goal <= 1.5 || tracking_progress_percentage_ < kThresPercentageOfArrival) &&
        flag_path_safe &&
        flag_robot_following_path &&
        !flag_path_deprecated){
      // no need plan, just publish old path
      // walkable_path_ptr_->header.stamp = ros::Time::now();
      pub_walkable_path_.publish(walkable_path_ptr_);

      // publish_robot_status_marker("following path");
      flag_planning_busy_ = false;
      return;
    }
    else{
      geometry_msgs::Point subgoal_pt;
      if(tracking_progress_percentage_ >= kThresPercentageOfArrival){
        // Subgoal arrival situation       
        subgoal_pt = generate_subgoal(localmap_ptr_, finalgoal_ptr_, tf_base2odom);
        ROS_WARN("Almost arrivied, generate new goal: (%.2f, %.2f)", subgoal_pt.x, subgoal_pt.y);
        publish_robot_status_marker("subgoal arrival, generate new subgoal");
      }else if(walkable_path_ptr_ && !flag_subgoal_safe){
        // Unsafe subgoal situation

        // Try to approach the unsafe subgoal but leave a safe distance
        subgoal_pt = approach_unsafe_subgoal(localmap_ptr_, walkable_path_ptr_, tf_odom2base);

        // If there is still no safe subgoal canidate from the old path, generate a new subgoal
        if(subgoal_pt.x == 0 && subgoal_pt.y == 0){
          subgoal_pt = generate_subgoal(localmap_ptr_, finalgoal_ptr_, tf_base2odom);
          ROS_WARN("subgoal is not safe, generate a new subgoal: (%.2f, %.2f)", subgoal_pt.x, subgoal_pt.y);
          publish_robot_status_marker("new subgoal is generated");
        }else{
          ROS_WARN("subgoal is not safe, choose a new subgoal from old path: (%.2f, %.2f)", subgoal_pt.x, subgoal_pt.y);
          publish_robot_status_marker("approach unsafe subgoal");
        }
      }else if(walkable_path_ptr_ && (!flag_path_safe || flag_path_deprecated)){
        // Unsafe path situation
        // Get subgoal from old path, and transform it to base frame
        std::vector<geometry_msgs::PoseStamped>::iterator it = walkable_path_ptr_->poses.begin();
        tf::Vector3 vec_raw(it->pose.position.x, it->pose.position.y, it->pose.position.z);
        tf::Vector3 vec_transformed = tf_odom2base * vec_raw;
        tf::pointTFToMsg(vec_transformed, subgoal_pt);
        if(!flag_path_safe){
          ROS_WARN("The old path is not safe, generate new path");
          publish_robot_status_marker("old path is not safe");
        }else{
          ROS_WARN("The old subgoal is too old, generate new path");
          publish_robot_status_marker("subgoal is too old");
        }
      }else if(walkable_path_ptr_ && !flag_robot_following_path){
        // Robot not following path situation
        subgoal_pt = generate_subgoal(localmap_ptr_, finalgoal_ptr_, tf_base2odom);
        ROS_WARN("The robot is not following the currenr path, generate new goal: (%.2f, %.2f)", subgoal_pt.x, subgoal_pt.y);
        publish_robot_status_marker("robot is not on the path");
      }else{
        // New plan situation
        subgoal_pt = generate_subgoal(localmap_ptr_, finalgoal_ptr_, tf_base2odom);
        ROS_WARN("There is no any old path existed, start plan...");
      }

      // A* path planning
      walkable_path_ptr_ = nav_msgs::Path::Ptr(new nav_msgs::Path());
      walkable_path_ptr_->header.frame_id = path_frame_id_;

      // coordinate to map grid
      // Trick: start plan from the grid which is in front of robot
      int origin_idx = std::round((-map_origin_y + path_start_offsety_) / map_resolution) * map_width + 
                std::round((-map_origin_x + path_start_offsetx_) / map_resolution);
      int map_x = std::round((subgoal_pt.x - map_origin_x) / map_resolution);
      int map_y = std::round((subgoal_pt.y - map_origin_y) / map_resolution);
      int target_idx = map_y * map_width + map_x;

      bool flag_success = path_solver_.FindPathByHashmap(localmap_ptr_, walkable_path_ptr_, origin_idx, target_idx, solver_timeout_ms_);
      if(flag_success){
        // Convert path from base_link coordinate to odom coordinate
        for(std::vector<geometry_msgs::PoseStamped>::iterator it = walkable_path_ptr_->poses.begin() ; it != walkable_path_ptr_->poses.end(); ++it) {
          // Walkable path topic without direction info
          tf::Vector3 vec_raw(it->pose.position.x, it->pose.position.y, it->pose.position.z);
          tf::Vector3 vec_transformed = rot_base2odom * vec_raw + trans_base2odom;
          tf::pointTFToMsg(vec_transformed, it->pose.position);
        }
        walkable_path_ptr_->header.stamp = ros::Time::now();
        pub_walkable_path_.publish(walkable_path_ptr_);
      }
      else{
        // Publish empty path if there are no path finding solution.
        ROS_ERROR("No solution for path finding in timeout: %.1f ms", solver_timeout_ms_);
        // walkable_path_ptr_->header.stamp = ros::Time::now();
        pub_walkable_path_.publish(walkable_path_ptr_);
      }

    }    
    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
  }

  flag_planning_busy_ = false;
}


void AstarPathfindingNode::sigint_cb(int sig) {
  ROS_INFO_STREAM("Node name: " << ros::this_node::getName() << " is shutdown.");
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
