#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

def callback(data):
    #get pose_node_x
    #get setpoint_x

def PID_X():
    rospy.init_node('pid_x')
    pid_pub_x = rospy.Publisher('control_x', Float64, queue_size=10)
    sub_pose = rospy.Subscriber('youbot/pose/x', Float64, callback)
    sub_setpoint = rospy.Subscriber('setpoint_x', Float64, callback)
    rospy.init_node('PID_X')
    while not rospy.is_shutdown():
        pid_pub_x.publish(vicon_data_x)

if __name__ == '__main__':
    PID_X()
