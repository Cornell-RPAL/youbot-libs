#!/usr/bin/env python3

import rospy, tf2_ros
from std_msgs.msg import Float64, PoseStamped

class PoseNode(object):

    def __init__(self):
        self.pub_x = rospy.Publisher('x_pid', Float64, queue_size=10)
        #self.pose_sub = rospy.Subscriber('vicon/youbot/pose', PoseStamped, callback)


if __name__ == '__main__':
  rospy.init_node('pose_node')
  P = PoseNode()
  #create a listener for the vicon pose information
  buffer = tf2_ros.Buffer()
  listen_x = tf2_ros.TransformListener(buffer)
  #should this be rospy.Time()?
  transform_x = buffer.lookupTransform(vicon.pose.position.x, 'x_pid', rospy.Time(0))
  self.pub_x.publish(transform_x)

  rospy.spin()
