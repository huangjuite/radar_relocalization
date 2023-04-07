#include "radar2D.hpp"



Radar2D::Radar2D(ros::NodeHandle* nh) {
  odom_topic_ = "/dopplerIO";
  amcl_pose_topic_ = "/amcl_pose";
  amcl_path_topic_ = "/amcl_path";
  global_frame_ = "/map";
  parent_frame_ = "/odom";
  child_frame_ = "/base_link";
  odomSub_ = nh->subscribe(odom_topic_, 1, &Radar2D::odomCb, this);
  amclPoseSub_ = nh->subscribe(amcl_pose_topic_, 1, &Radar2D::amclPoseCb, this);
  amclPathPub_ = nh->advertise<nav_msgs::Path>(amcl_path_topic_, 1000);
  amcl_path_.header.frame_id = global_frame_;
  // initPosePub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10);
  // setInitial();
}

void Radar2D::setInitial() {
  /*
    Double Bag initial pose
    pose: 
    position: 
      x: 52.57637405395508
      y: -3.3849635124206543
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.999865070137846
      w: 0.016426853570914664
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
  */
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = global_frame_;
  pose.header.stamp = ros::Time::now(); 

  geometry_msgs::Point point;
  geometry_msgs::Quaternion quat;
  point.x = 52.57637405395508;
  point.y = -3.3849635124206543;
  point.z = 0.0;

  quat.x = 0.0;
  quat.y = 0.0;
  quat.z = 0.999865070137846;
  quat.w = 0.016426853570914664;

  pose.pose.pose.position = point;
  pose.pose.pose.orientation = quat;

  pose.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25,
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787};
  
  initPosePub_.publish(pose);
}

void Radar2D::amclPoseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcl_pose) {
  geometry_msgs::PoseStamped pathPose;
  pathPose.pose = amcl_pose->pose.pose;
  amcl_path_.poses.push_back(pathPose);
  amclPathPub_.publish(amcl_path_);
}


void Radar2D::odomCb(const nav_msgs::OdometryConstPtr& odom) {
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
}

int main(int argc, char** argv )
{
    ros::init(argc, argv, "Radar2D_node");

    ros::NodeHandle nh("");
    
    Radar2D Radar2D(&nh);
    
    ros::spin();
    return 0;
}