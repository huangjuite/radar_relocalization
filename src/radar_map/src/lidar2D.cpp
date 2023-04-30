#include "lidar2D.hpp"

Lidar2D::Lidar2D(ros::NodeHandle *nh)
{
    odom_topic_ = "/odometry/imu_map";
    amcl_pose_topic_ = "/amcl_pose";
    amcl_path_topic_ = "/amcl_path";
    dense_cloud_topic_ = "/dense_cloud";
    global_frame_ = "/map";
    parent_frame_ = "/odom";
    child_frame_ = "/base_link";

    sub_odom = nh->subscribe(odom_topic_, 100, &Lidar2D::cb_odom, this);
    amclPoseSub_ = nh->subscribe(amcl_pose_topic_, 1, &Lidar2D::amclPoseCb, this);
    amclPathPub_ = nh->advertise<nav_msgs::Path>(amcl_path_topic_, 1000);
    amcl_path_.header.frame_id = global_frame_;
    pose_cap_ = 10;
}

void Lidar2D::cb_odom(const nav_msgs::OdometryConstPtr &odom_msg)
{
    // Get odom->base_link from odometry message
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = odom_msg->header.stamp;
    transformStamped.header.frame_id = parent_frame_;
    transformStamped.child_frame_id = child_frame_;
    transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
    transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
    transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
    transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x;
    transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y;
    transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z;
    transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w;
    odom2BaseBr_.sendTransform(transformStamped);
}

void Lidar2D::amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_pose)
{
    geometry_msgs::PoseStamped pathPose;
    pathPose.pose = amcl_pose->pose.pose;
    pathPose.header.stamp = amcl_pose->header.stamp;
    amcl_path_.poses.push_back(pathPose);
    amclPathPub_.publish(amcl_path_);
}

Lidar2D::~Lidar2D()
{
    std::ofstream myfile;
    myfile.open("/home/rpl/radar_relocalization/lidar_results.csv");
    myfile << "stamp, x, y, theta\n";

    for (auto p : amcl_path_.poses)
    {
        Eigen::Affine3d a3d;
        Eigen::fromMsg(p.pose, a3d);
        Eigen::Vector3d ypr = a3d.rotation().eulerAngles(2, 1, 0);
        myfile << p.header.stamp.toSec() << "," << p.pose.position.x << ", "
               << p.pose.position.y << ", " << ypr(0) << "\n ";
    }

    myfile.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Lidar2D_node");

    ros::NodeHandle nh("");

    Lidar2D lidar2d(&nh);

    ros::spin();
    return 0;
}