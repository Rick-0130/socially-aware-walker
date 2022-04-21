#include <iostream>
#include <time.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <cv_bridge/cv_bridge.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// Custom msg & srv
#include <walker_msgs/Detection2DTrigger.h>
#include <walker_msgs/Det3D.h>
#include <walker_msgs/Det3DArray.h>

// Message filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// Eigen
#include <Eigen/Dense>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// TF
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h> // Centroid
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h> // ros2pcl
#include <pcl/filters/radius_outlier_removal.h> // RemoveOutlier
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace cv;

typedef message_filters::sync_policies::ApproximateTime<cv_bridge::CvImage, sensor_msgs::PointCloud2> MySyncPolicy;
typedef message_filters::Synchronizer<MySyncPolicy> MySynchronizer;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

// Just for color words display
static const string COLOR_RED = "\e[0;31m";
static const string COLOR_GREEN = "\e[0;32m";
static const string COLOR_YELLOW = "\e[0;33m"; 
static const string COLOR_NC = "\e[0m";

static const int kNumOfInterestClass = 1;
static const string kInterestClassNames[kNumOfInterestClass] = {"person"};

template <typename T, typename A>
int arg_max(std::vector<T, A> const& vec) {
    return static_cast<int>(std::distance(vec.begin(), max_element(vec.begin(), vec.end())));
}

template <typename T, typename A>
int arg_min(std::vector<T, A> const& vec) {
    return static_cast<int>(std::distance(vec.begin(), min_element(vec.begin(), vec.end())));
}

class ObjInfo {
public:
    ObjInfo(){
        cloud = PointCloudXYZPtr(new PointCloudXYZ);
        radius = 0.0;
    }
    walker_msgs::BBox2D box;
    PointCloudXYZPtr cloud;
    geometry_msgs::Point location;
    double radius;
    // geometry_msgs::Point dimension;
};


class ScanImageCombineNode {
public:
    ScanImageCombineNode(ros::NodeHandle nh, ros::NodeHandle pnh);
    void img_scan_cb(const cv_bridge::CvImage::ConstPtr &cv_ptr, const sensor_msgs::PointCloud2ConstPtr &cloud_raw_ptr);
    void separate_outlier_points(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out);
    void odom_transformed(tf::Vector3 &input,tf::Vector3 &output,tf::StampedTransform tf_base2odom_);
    bool is_interest_class(string class_name);
    cv::Point2d point_pointcloud2pixel(double x_from_camera, double y_from_camera, double z_from_camera);

    // Transformation
    cv::Mat K_;
    cv::Mat D_;

    // ROS related
    ros::NodeHandle nh_, pnh_;
    tf::TransformListener tf_listener_;
    ros::Publisher pub_combined_image_;
    ros::Publisher pub_marker_array_;
    ros::Publisher pub_colored_pc_;
    ros::Publisher pub_detection3d_;
    ros::ServiceClient yolov4_detect_;  // ROS Service client

    // TF listener
    tf::TransformListener* tflistener_ptr_;
    tf::StampedTransform tf_base2odom_;    

    string localmap_frameid_;

    // Message filters
    message_filters::Subscriber<sensor_msgs::PointCloud2> scan_sub_;
    message_filters::Subscriber<cv_bridge::CvImage> image_sub_;
    boost::shared_ptr<MySynchronizer> sync_;

    // Object list
    std::vector<ObjInfo> obj_list;
};


ScanImageCombineNode::ScanImageCombineNode(ros::NodeHandle nh, ros::NodeHandle pnh): nh_(nh), pnh_(pnh) {
    // ROS parameters
    string scan_topic;
    string img_topic;
    string caminfo_topic;
    string yolo_srv_name = "yolov4_node/yolo_detect";
    ros::param::param<string>("~scan_topic", scan_topic, "/scan_pointcloud_filtered");
    ros::param::param<string>("~img_topic", img_topic, "/camera1/color/image_raw"); 
    ros::param::param<string>("~caminfo_topic", caminfo_topic, "/camera1/color/camera_info"); 
    ros::param::param<string>("~localmap_frameid", localmap_frameid_, "base_link");

    // ROS publisher & subscriber & message filter
    pub_combined_image_ = nh_.advertise<sensor_msgs::Image>("debug_reprojection", 1);
    pub_marker_array_ = nh.advertise<visualization_msgs::MarkerArray>("obj_marker", 1);
    pub_colored_pc_ = nh.advertise<sensor_msgs::PointCloud2>("colored_pc", 1);
    pub_detection3d_ = nh.advertise<walker_msgs::Det3DArray>("det3d_result", 1);
    scan_sub_.subscribe(nh_, scan_topic, 1);
    image_sub_.subscribe(nh_, img_topic, 1);
    sync_.reset(new MySynchronizer(MySyncPolicy(10), image_sub_, scan_sub_));
    sync_->registerCallback(boost::bind(&ScanImageCombineNode::img_scan_cb, this, _1, _2));

    // ROS service client
    ROS_INFO_STREAM("Wait for yolo detection service in 30 seconds...");
    if(!ros::service::waitForService(yolo_srv_name, ros::Duration(30.0))) {
        ROS_ERROR("Cannot get the detection service: %s. Aborting...", yolo_srv_name.c_str());
        exit(-1);
    }
    yolov4_detect_ = nh_.serviceClient<walker_msgs::Detection2DTrigger>(yolo_srv_name);


    // Prepare the transformation matrix from odom to base
    tflistener_ptr_ = new tf::TransformListener();
    ROS_INFO("Wait for TF from odom to %s in 10 seconds...", localmap_frameid_.c_str());
    try{
        tflistener_ptr_->waitForTransform( "odom", localmap_frameid_,
                                    ros::Time(), ros::Duration(5));
        tflistener_ptr_->lookupTransform( "odom", localmap_frameid_,
                                    ros::Time(), tf_base2odom_);
        ROS_INFO("Done.");
    }
    catch (tf::TransformException ex){
        ROS_ERROR("\nCannot get TF from odom to %s: %s. Aborting...", localmap_frameid_.c_str(), ex.what());
        exit(-1);
    }

    // Prepare intrinsic matrix
    boost::shared_ptr<sensor_msgs::CameraInfo const> caminfo_ptr;
    double fx, fy, cx, cy;
    double k1, k2, p1, p2;
    ROS_INFO_STREAM("Wait for camera_info message in 10 seconds");
    caminfo_ptr = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(caminfo_topic, ros::Duration(10.0));
    if(caminfo_ptr != NULL){       
        fx = caminfo_ptr->P[0];
        fy = caminfo_ptr->P[5];
        cx = caminfo_ptr->P[2];
        cy = caminfo_ptr->P[6];

        k1 = caminfo_ptr->D[0];
        k2 = caminfo_ptr->D[1];
        p1 = caminfo_ptr->D[2];
        p2 = caminfo_ptr->D[3];

    }else {
        ROS_INFO_STREAM("No camera_info received, use default values");
        fx = 384.396;
        fy = 384.011;
        cx = 316.854;
        cy = 245.37;

        k1 = -0.0556167;
        k2 = 0.0649698;
        p1 = -0.000873693;
        p2 = -0.000543307;
    }
    K_ = (Mat_<double>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.);
    D_ = (Mat_<double>(5, 1) << k1, k2, p1, p2, 0.0);
    // cout << "K:\n" << K_ << endl;
    // cout << "D:\n" << D_ << endl;

    cout << COLOR_GREEN << ros::this_node::getName() << " is ready." << COLOR_NC << endl;
}


void ScanImageCombineNode::separate_outlier_points(PointCloudXYZPtr cloud_in, PointCloudXYZPtr cloud_out) {
    // Euclidean Cluster Extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euler_extractor;
    euler_extractor.setClusterTolerance(0.3);
    euler_extractor.setMinClusterSize(1);
    euler_extractor.setMaxClusterSize(5000);  // need to check the max pointcloud size of each object
    euler_extractor.setSearchMethod(tree);
    euler_extractor.setInputCloud(cloud_in);
    euler_extractor.extract(cluster_indices);

    // Find the cloud cluster which is closest to ego
    int idx_proper_cloud = 0;
    std::vector<float> candidates;
    for(int i = 0; i < cluster_indices.size(); i++) {
        Eigen::Vector3f centroid;
        centroid << 0, 0, 0;
        for(int j = 0; j < cluster_indices[i].indices.size(); j++) {
            centroid[0] += cloud_in->points[cluster_indices[i].indices[j]].x;
            centroid[1] += cloud_in->points[cluster_indices[i].indices[j]].y;
            centroid[2] += cloud_in->points[cluster_indices[i].indices[j]].z;
        }
        centroid /= cluster_indices[i].indices.size();
        candidates.push_back(centroid.norm());
    }
    

   
    idx_proper_cloud = arg_min(candidates);
    
    // ROS_ERROR("Proper candidates: %.2f\n", candidates[idx_proper_cloud]);
    
    pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices(cluster_indices[idx_proper_cloud]));
    PointCloudXYZPtr cloud_extracted(new PointCloudXYZ);
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(cloud_in);
    extractor.setIndices(inliers_ptr);
    extractor.setNegative(false);
    extractor.filter(*cloud_extracted);

    // Remove outlier
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(cloud_extracted);
    outrem.setRadiusSearch(0.2);
    outrem.setMinNeighborsInRadius(2);
    outrem.filter(*(cloud_out));

    pcl::copyPointCloud(*cloud_in, *cloud_out);                  
}


bool ScanImageCombineNode::is_interest_class(string class_name){
    for(int i = 0; i < kNumOfInterestClass; i++) {
        if(strcmp(class_name.c_str(), kInterestClassNames[i].c_str()) == 0)
            return true;
    }
    return false;
}





cv::Point2d ScanImageCombineNode::point_pointcloud2pixel(double x_from_camera, double y_from_camera, double z_from_camera) {
    // Transform to camera frame
    tf::Vector3 pt_camframe(x_from_camera, y_from_camera, z_from_camera); 
    if(pt_camframe.getZ() <= 0.0) // points behind ego
        return cv::Point2d(-1, -1);

    // Normalization: z --> 1
    pt_camframe.setX(pt_camframe.getX() / pt_camframe.getZ());
    pt_camframe.setY(pt_camframe.getY() / pt_camframe.getZ());
    pt_camframe.setZ(1.0);  

    // Trasform to pixel frame
    cv::Mat uv = K_ * (cv::Mat_<double>(3, 1) << pt_camframe.getX(), pt_camframe.getY(), pt_camframe.getZ());
    cv::Point2d pt_pixelframe(uv.at<double>(0, 0), uv.at<double>(1, 0));

    return pt_pixelframe;
}

void ScanImageCombineNode::odom_transformed(tf::Vector3 &input,tf::Vector3 &output,tf::StampedTransform tf_base2odom_){
    
    output = tf_base2odom_ * input;
    
    ///tf::Vector3 trans_base2odom = tf_base2odom_.getOrigin();
    ///tf::Matrix3x3 rot_base2odom = tf_base2odom_.getBasis();
    // tf::Matrix3x3 rot_base2odom = tf_base2odom_.getBasis().transpose();
    // tf::Vector3 trans_base2odom = rot_base2odom * tf_base2odom_.getOrigin();
    // trans_base2odom .setY(-trans_base2odom.getY());
    //cout<<trans_base2odom.getX()<<endl;
    //output=rot_base2odom*input+trans_base2odom;
    //cout<<"input"<<input.getY()<<endl;
    //cout<<"input.x"<<input.getX()<<endl;
    //cout<<"input.y"<<input.getY()<<endl;
    // cout<<"input.z"<<input.getZ()<<endl;
    //cout<<"output.x"<<output.getX()<<endl;
    //cout<<"output.y"<<output.getY()<<endl;
    // cout<<"output.z"<<output.getZ()<<endl;
    
}

void ScanImageCombineNode::img_scan_cb(const cv_bridge::CvImage::ConstPtr &cv_ptr,  const sensor_msgs::PointCloud2ConstPtr &cloud_raw_ptr){
    //cout <<"first"<< ros::Time::now()-cloud_raw_ptr->header.stamp<<endl;
    // Object list init
    obj_list.clear();
    visualization_msgs::MarkerArray marker_array;
    
    // 2D bounding box detection service
    walker_msgs::Detection2DTrigger srv;
    srv.request.image = *(cv_ptr->toImageMsg());
    //0.07 delay
    if(!yolov4_detect_.call(srv)){
        ROS_ERROR("Failed to call service");
        return;
    }
    //cout <<"second"<< ros::Time::now()-cloud_raw_ptr->header.stamp<<endl;
    // Collect all interest classes to obj_list
    std::vector<walker_msgs::BBox2D> boxes = srv.response.result.boxes;
    for(int i = 0; i < boxes.size(); i++) {
        if(is_interest_class(boxes[i].class_name)) {
            ObjInfo obj_info;
            obj_info.box = boxes[i];
            // Skip the boxes which are too closed
            bool flag_too_closed = false;

            obj_list.push_back(obj_info);
            
        }
    }
    
    // Reconstruct undistorted cvimage from detection result image
    cv::Mat cvimage;
    cv_bridge::CvImagePtr detected_cv_ptr = cv_bridge::toCvCopy(srv.response.result.result_image);
    
    //cv::undistort(detected_cv_ptr->image, cvimage, K_, D_); //0.02 delay
    // Convert  pointcloud:   ROS PointCloud2 --> PCL PointCloudXYZ
    PointCloudXYZPtr cloud_msg(new PointCloudXYZ); 
    pcl::fromROSMsg(*cloud_raw_ptr, *cloud_msg);
    
    // Color pointcloud to visaulize detected points
    PointCloudXYZRGBPtr cloud_colored(new PointCloudXYZRGB);
    // Convert laserscan points to pixel points
    std::vector<cv::Point2d> pts_uv;

    
    
    
    //ROS_INFO("Wait for TF from odom to %s in 10 seconds...", localmap_frameid_.c_str());
    try{
        tflistener_ptr_->waitForTransform("odom", localmap_frameid_,
                                    ros::Time(0), ros::Duration(0.025));
        tflistener_ptr_->lookupTransform("odom", localmap_frameid_,
                                    ros::Time(0), tf_base2odom_);
        //ROS_INFO("Done.");
    }
    catch (tf::TransformException ex){
        //ROS_ERROR("\nCannot get TF from odom to %s: %s. Aborting...", localmap_frameid_.c_str(), ex.what());
        exit(-1);
    }
    
    //cout<<cloud_msg->points.size()<<endl;
    for (int i = 0; i < cloud_msg->points.size(); i++) {
        cv::Point2d pt_uv = point_pointcloud2pixel(cloud_msg->points[i].x, cloud_msg->points[i].y, cloud_msg->points[i].z); 
        if(pt_uv.x == -1 && pt_uv.y == -1)
            continue;
        pts_uv.push_back(pt_uv);
        // Connect relationship between valid laserscan points to interest classes
        for(int j = 0; j < obj_list.size(); j++) {
            float diff_x = fabs((float)(pt_uv.x - obj_list[j].box.center.x));
            float diff_y = fabs((float)(pt_uv.y - obj_list[j].box.center.y));
            if(diff_x < obj_list[j].box.size_x / 4 && diff_y < obj_list[j].box.size_y / 4) {
                 obj_list[j].cloud->points.push_back(cloud_msg->points[i]);    
            }
            // Note that the pointcloud would be registered repeatly, so need to filter it later.
        }        
    }

    // Custom message
    walker_msgs::Det3DArray detection_array;
    
    // Remove outlier for each object cloud
    for(int i = 0; i < obj_list.size(); i++) {
        if(obj_list[i].cloud->points.size() > 1){
            //cout<<obj_list[i].cloud->points.size()<<endl;
            // Clustering and outlier removing
            
            separate_outlier_points(obj_list[i].cloud, obj_list[i].cloud);
            if(obj_list[i].cloud->points.size() < 1)
                continue;

            // Merge raw detected points with color to visualization
            if(pub_colored_pc_.getNumSubscribers() > 0) {
                int color_r = (rand() % 5) * 60;
                int color_g = (rand() % 5) * 60;
                int color_b = (rand() % 5) * 60;
                PointCloudXYZRGBPtr tmp_cloud(new PointCloudXYZRGB);
                pcl::copyPointCloud(*(obj_list[i].cloud), *tmp_cloud);
                for(auto& point: *tmp_cloud) {
                    point.r = color_r;
                    point.g = color_g;
                    point.b = color_b;
                }
                *cloud_colored += *tmp_cloud;
            }

            // Find the center of each object
            pcl::PointXYZ min_point, max_point;
            Eigen::Vector3f center;
            pcl::getMinMax3D(*(obj_list[i].cloud), min_point, max_point);
            center = (min_point.getVector3fMap() + max_point.getVector3fMap()) / 2.0;
            obj_list[i].location.x = center[0];
            obj_list[i].location.y = center[1];
            obj_list[i].location.z = center[2];
            tf::Vector3 position_vec(obj_list[i].location.z+0.41,
                                     -obj_list[i].location.x, 
                                     obj_list[i].location.y);
            tf::Vector3 position_vec1;
            odom_transformed(position_vec,position_vec1,tf_base2odom_);
            obj_list[i].location.x=position_vec1.getX();
            obj_list[i].location.y=position_vec1.getY();
            obj_list[i].location.z=position_vec1.getZ();
            double tmp_r1 = sqrt(pow(max_point.x - center[0], 2) + pow(max_point.y - center[1], 2));
            double tmp_r2 = sqrt(pow(min_point.x - center[0], 2) + pow(min_point.y - center[1], 2));
            obj_list[i].radius = std::max(tmp_r1, tmp_r2);


            // Pack the custom ros package
            walker_msgs::Det3D det_msg;
            det_msg.x = obj_list[i].location.x;
            det_msg.y = obj_list[i].location.y;
            det_msg.z = obj_list[i].location.z;
            
            
            //cout<< "x0:" << det_msg.x<< endl;
            //cout<< "y0:" << det_msg.y << endl;
            det_msg.yaw = 0;
            det_msg.radius = obj_list[i].radius;
            det_msg.confidence = obj_list[i].box.score;
            det_msg.class_name = obj_list[i].box.class_name;
            det_msg.class_id = obj_list[i].box.id;
            detection_array.dets_list.push_back(det_msg);

            // Visualization
            visualization_msgs::Marker marker;
            marker.header.frame_id = cloud_raw_ptr->header.frame_id;
            marker.header.stamp = cloud_raw_ptr->header.stamp;
            marker.ns = "detection_result";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.lifetime = ros::Duration(0.2);
            // marker.lifetime = ros::Duration(10.0);
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = center[0];
            marker.pose.position.y = center[1];
            marker.pose.position.z = center[2];
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            double distance =sqrt(obj_list[i].location.x*obj_list[i].location.x+obj_list[i].location.z*obj_list[i].location.z);
            //cout<< "d:" << distance << endl;
            //cout<< "x:" << obj_list[i].location.x << endl;
            //cout<< "z:" << obj_list[i].location.z << endl;
            // marker.scale.x = marker.scale.y = obj_list[i].radius * 2;
            marker.scale.x = 0.8;
            marker.scale.y = 1.6;
            marker.scale.z = 0.8;
            marker.color.a = 0.2;
            marker.color.g = 1.0;
            marker_array.markers.push_back(marker);
        }
        // TODO: deal with the objects which has no matched points
    }
    
    
    // Publish visualization topics
    if(pub_marker_array_.getNumSubscribers() > 0)
        pub_marker_array_.publish(marker_array);
    if(pub_colored_pc_.getNumSubscribers() > 0) {
        sensor_msgs::PointCloud2 colored_cloud_msg;
        pcl::toROSMsg(*cloud_colored, colored_cloud_msg);
        colored_cloud_msg.header.frame_id = cloud_raw_ptr->header.frame_id;
        pub_colored_pc_.publish(colored_cloud_msg);
    }
    if(pub_combined_image_.getNumSubscribers() > 0){
        // Draw points in images
        for (int j = 0; j < pts_uv.size(); ++j)
            cv::circle(cvimage, pts_uv[j], 1, Scalar(0, 255, 0), 1);
        cv_bridge::CvImage result_image(cv_ptr->header, "rgb8", cvimage);
        pub_combined_image_.publish(result_image.toImageMsg());
    }

    // Publish detection result
    if(pub_detection3d_.getNumSubscribers() > 0) {
        detection_array.header.frame_id = cloud_raw_ptr->header.frame_id;
        detection_array.header.stamp = ros::Time::now();
        detection_array.pointcloud = *cloud_raw_ptr;
        pub_detection3d_.publish(detection_array);
    }
    
    //cout <<"third"<< ros::Time::now()-cloud_raw_ptr->header.stamp<<endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_clustering_node");
    ros::NodeHandle nh, pnh("~");
    ScanImageCombineNode node(nh, pnh);
    ros::spin();
    return 0;
}
