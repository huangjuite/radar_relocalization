#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/LaserScan.h>
#include <deque>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <Eigen/Dense>
#include <tf2_eigen/tf2_eigen.h>
#include <signal.h>

#include "utils.hpp"

#include <iostream>
#include <fstream>

class Lidar2D
{
public:
    Lidar2D(ros::NodeHandle *nh);
    ~Lidar2D();

private:
    ros::Subscriber amclPoseSub_, sub_odom;
    ros::Publisher amclPathPub_, initPosePub_;

    // PointCloud stacking
    ros::Publisher densePointPub_;
    std::deque<Eigen::Matrix4f> transform_queue_;
    std::deque<pcl_cloud_ptr_t> cloud_queue_;
    int pose_cap_; // How many poses to store in an array before stacking

    tf2_ros::TransformBroadcaster odom2BaseBr_;
    std::string parent_frame_; // odom
    std::string child_frame_;  // base
    std::string global_frame_; // map
    std::string odom_topic_, amcl_pose_topic_, amcl_path_topic_, radar_topic_, dense_cloud_topic_;
    nav_msgs::Path amcl_path_;

    void amclPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl_pose);
    void cb_odom(const nav_msgs::OdometryConstPtr &odom_msg);
};