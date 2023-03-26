#include "odom2tf.hpp"



Odom2Tf::Odom2Tf(ros::NodeHandle* nh) {
  odom_topic_ = "/dopplerIO";
  parent_frame_ = "/odom";
  child_frame_ = "/base_link";
  odomSub_ = nh->subscribe(odom_topic_, 1, &Odom2Tf::odomCb, this);
}


void Odom2Tf::odomCb(const nav_msgs::OdometryConstPtr& odom) {
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
    ros::init(argc, argv, "odom2Tf_node");

    ros::NodeHandle nh("");
    
    Odom2Tf odom2tf(&nh);
    
    ros::spin();
    return 0;
}