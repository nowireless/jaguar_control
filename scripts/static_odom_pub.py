#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry

rospy.init_node('static_joint_state_pub', anonymous=True)
pub = rospy.Publisher('/mobile_base_controller/odom', Odometry, queue_size=10)

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    msg = Odometry()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_link"
    pub.publish(msg)
    rate.sleep()
