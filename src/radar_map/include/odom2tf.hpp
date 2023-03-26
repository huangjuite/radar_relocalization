#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>

class Odom2Tf {
public:
    Odom2Tf(ros::NodeHandle* nh);
private:
    ros::Subscriber odomSub_;
    tf2_ros::TransformBroadcaster odom2BaseBr_;
    std::string parent_frame_; //odom
    std::string child_frame_; // base
    std::string odom_topic_;

    void odomCb(const nav_msgs::OdometryConstPtr& odom);
};