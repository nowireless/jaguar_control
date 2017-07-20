#!/usr/bin/python
import rospy
from sensor_msgs.msg import JointState

rospy.init_node('static_joint_state_pub', anonymous=True)
pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

joint_name = rospy.get_param('~joint')
pos = rospy.get_param('~pos')
vel = rospy.get_param('~vel')
effort = rospy.get_param('~effort')
update_rate = rospy.get_param('~rate')

rate = rospy.Rate(update_rate)

while not rospy.is_shutdown():
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name = [joint_name]
    msg.position = [pos]
    msg.velocity = [vel]
    msg.effort = [effort]

    pub.publish(msg)

    rate.sleep()
