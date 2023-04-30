#include "radar2D.hpp"

Radar2D::Radar2D(ros::NodeHandle *nh)
{
    odom_topic_ = "/dopplerIO";
    amcl_pose_topic_ = "/amcl_pose";
    amcl_path_topic_ = "/amcl_path";
    dense_cloud_topic_ = "/dense_cloud";
    radar_topic_ = "/radar_inlier";
    global_frame_ = "/map";
    parent_frame_ = "/odom";
    child_frame_ = "/base_link";

    odomSub_.subscribe(*nh, odom_topic_, 20);
    radarSub_.subscribe(*nh, radar_topic_, 20);
    sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(20), radarSub_, odomSub_);
    sync->registerCallback(boost::bind(&Radar2D::cb_radar, this, _1, _2));
    denseCloudPub_ = nh->advertise<sensor_msgs::PointCloud2>(dense_cloud_topic_, 1000);

    amclPoseSub_ = nh->subscribe(amcl_pose_topic_, 1, &Radar2D::amclPoseCb, this);
    amclPathPub_ = nh->advertise<nav_msgs::Path>(amcl_path_topic_, 1000);
    amcl_path_.header.frame_id = global_frame_;
    pose_cap_ = 10;
}

void Radar2D::broadcastOdom(const nav_msgs::OdometryConstPtr &odom_msg)
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

void Radar2D::cb_radar(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const nav_msgs::OdometryConstPtr &odom_msg)
{
    broadcastOdom(odom_msg);
    addQueue(pc_msg, odom_msg);
    pcl_cloud_ptr_t dense_cloud_ptr = getDenseCloud(cloud_queue_, transform_queue_);
    // Convert to ROS data type
    sensor_msgs::PointCloud2 densePointCloud2;
    pcl::toROSMsg(*dense_cloud_ptr, densePointCloud2);
    densePointCloud2.header = pc_msg->header;
    denseCloudPub_.publish(densePointCloud2);
}

void Radar2D::addQueue(const sensor_msgs::PointCloud2ConstPtr &pc_msg, const nav_msgs::OdometryConstPtr &odom_msg)
{
    // Transform odom and ROS point cloud to appropriate datatypes, store them in queues.
    // Add newest odom to pose_arr_, check if it has exceeded limit
    Eigen::Matrix4f m = toMatrix(odom_msg->pose.pose);
    transform_queue_.push_back(m);
    // Check if pose_arr_.size exceeds capacity, if so perform queue.pop
    if (transform_queue_.size() > pose_cap_)
    {
        transform_queue_.pop_front();
    }

    // Transfrom PointCloud2 to PCL
    pcl_cloud_ptr_t pcl_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*pc_msg, *pcl_cloud_ptr);
    cloud_queue_.push_back(pcl_cloud_ptr);
    if (cloud_queue_.size() > pose_cap_)
    {
        cloud_queue_.pop_front();
    }
}

void Radar2D::amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_pose)
{
    geometry_msgs::PoseStamped pathPose;
    pathPose.pose = amcl_pose->pose.pose;
    pathPose.header.stamp = amcl_pose->header.stamp;
    amcl_path_.poses.push_back(pathPose);
    amclPathPub_.publish(amcl_path_);
}

Radar2D::~Radar2D()
{
    std::ofstream myfile;
    myfile.open("/home/rpl/radar_relocalization/radar_results.csv");
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
    ros::init(argc, argv, "Radar2D_node");

    ros::NodeHandle nh("");

    Radar2D Radar2D(&nh);

    ros::spin();
    return 0;
}