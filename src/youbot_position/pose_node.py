#!/usr/bin/env python3

import rospy, tf2_ros
from std_msgs.msg import Float64, PoseStamped

class PoseNode(object):

    def __init__(self):
        self.pub_x = rospy.Publisher('x_pid', Float64, queue_size=10)
        #self.pose_sub = rospy.Subscriber('vicon/youbot/pose', PoseStamped, callback)
        self.buffer = tf2_ros.Buffer()
        self.listen_x = tf2_ros.TransformListener(buffer)

    def callback(self, data):
        transform_x = buffer.lookupTransform( 'x_pid', rospy.Time(0))
        self.pub_x.publish(transform_x)

if __name__ == '__main__':
  rospy.init_node('pose_node')
  P = PoseNode()
  rospy.spin()
