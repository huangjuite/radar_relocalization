#! /usr/bin/env python3

import rospy
import numpy as np
import pandas as pd
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped

rospy.init_node('sub_odom')

data = []


def cb_odom(msg: PoseWithCovarianceStamped):
    pos = msg.pose.pose.position
    o = msg.pose.pose.orientation
    euler = euler_from_quaternion([o.x, o.y, o.z, o.w])
    theta = euler[-1]
    cov = np.asarray(msg.pose.covariance).reshape(6, 6)
    data.append([msg.header.stamp.to_sec(), pos.x, pos.y, o.x, o.y, o.z, o.w, cov[0, 0], cov[1, 1], cov[5, 5]])


def on_shutdown():
    df = pd.DataFrame(data, columns=['stamp', 'x', 'y', 'qx', 'qy', 'qz', 'qw', 'var_x', 'var_y', 'var_theta'])
    print(df)
    df.to_csv('/home/rpl/radar_relocalization/odom.csv')


sub = rospy.Subscriber(
    "/amcl_pose", PoseWithCovarianceStamped, cb_odom, queue_size=10)
rospy.on_shutdown(on_shutdown)

rospy.spin()
