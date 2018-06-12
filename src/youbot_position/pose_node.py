#!/usr/bin/env python2.7

import rospy
import tf2_ros
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped

class PoseNode(object):

    def __init__(self):
        self.pub_x = rospy.Publisher('x_pid', Float64, queue_size=10)
        #self.pose_sub = rospy.Subscriber('vicon/youbot/pose', PoseStamped, callback)

    def broadcaster(self):
      buffer = tf2_ros.Buffer()
      listen_x = tf2_ros.TransformListener(buffer)
      transform_x = buffer.lookup_transform('/base_footprint', '/base_footprint', rospy.Time(0))
      self.pub_x.publish(transform_x.transform.translation.x)

if __name__ == '__main__':
  rospy.init_node('pose_node')
  P = PoseNode()
  P.broadcaster()
  rospy.spin()
