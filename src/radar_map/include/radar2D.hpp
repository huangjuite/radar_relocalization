#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class Radar2D {
public:
    Radar2D(ros::NodeHandle* nh);
private:
    ros::Subscriber odomSub_;
    ros::Subscriber amclPoseSub_;
    ros::Publisher amclPathPub_, initPosePub_;
    tf2_ros::TransformBroadcaster odom2BaseBr_;
    std::string parent_frame_; //odom
    std::string child_frame_; // base
    std::string global_frame_; // map
    std::string odom_topic_, amcl_pose_topic_, amcl_path_topic_;
    nav_msgs::Path amcl_path_;

    void odomCb(const nav_msgs::OdometryConstPtr& odom);
    void amclPoseCb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& amcl_pose);
    void setInitial();
};