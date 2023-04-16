#include "radar2D.hpp"

Radar2D::Radar2D(ros::NodeHandle *nh)
{
  odom_topic_ = "/dopplerIO";
  amcl_pose_topic_ = "/amcl_pose";
  amcl_path_topic_ = "/amcl_path";
  radar_topic_ = "/radar_inlier";
  global_frame_ = "/map";
  parent_frame_ = "/odom";
  child_frame_ = "/base_link";

  odomSub_.subscribe(*nh, odom_topic_, 20);
  radarSub_.subscribe(*nh, radar_topic_, 20);
  sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(20), radarSub_, odomSub_);
  sync->registerCallback(boost::bind(&Radar2D::cb_radar, this, _1, _2));

  amclPoseSub_ = nh->subscribe(amcl_pose_topic_, 1, &Radar2D::amclPoseCb, this);
  amclPathPub_ = nh->advertise<nav_msgs::Path>(amcl_path_topic_, 1000);
  amcl_path_.header.frame_id = global_frame_;
  pose_cap_ = 10;
  // initPosePub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
  // setInitial();
}

void Radar2D::cb_radar(const sensor_msgs::PointCloud2ConstPtr pc_msg, const nav_msgs::OdometryConstPtr odom)
{
  
}

void Radar2D::amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_pose)
{
  geometry_msgs::PoseStamped pathPose;
  pathPose.pose = amcl_pose->pose.pose;
  amcl_path_.poses.push_back(pathPose);
  amclPathPub_.publish(amcl_path_);
}

void Radar2D::odomCb(const nav_msgs::OdometryConstPtr &odom)
{
  // Get odom->base_link from odometry message
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = odom->header.stamp;
  transformStamped.header.frame_id = parent_frame_;
  transformStamped.child_frame_id = child_frame_;
  transformStamped.transform.translation.x = odom->pose.pose.position.x;
  transformStamped.transform.translation.y = odom->pose.pose.position.y;
  transformStamped.transform.translation.z = odom->pose.pose.position.z;
  transformStamped.transform.rotation.x = odom->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = odom->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = odom->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = odom->pose.pose.orientation.w;
  odom2BaseBr_.sendTransform(transformStamped);

  // Add newest odom to pose_arr_, check if it has exceeded limit
  Eigen::Matrix4f m = toMatrix(odom->pose.pose);
  pose_mat_.push_back(m);
  // Check if pose_arr_.size exceeds capacity, if so perform queue.pop
  if (pose_mat_.size() > pose_cap_)
  {
    pose_mat_.pop_front();
    // Perform point cloud stacking
  }
}

Eigen::Matrix4f Radar2D::toMatrix(geometry_msgs::Pose pose)
{
  Eigen::Matrix4d m;
  Eigen::Affine3d a3d;
  Eigen::fromMsg(pose, a3d);
  m = a3d.matrix();
  return m.cast<float>();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Radar2D_node");

  ros::NodeHandle nh("");

  Radar2D Radar2D(&nh);

  ros::spin();
  return 0;
}