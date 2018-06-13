#!/usr/bin/env python2.7

import rospy
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped


class PoseNode(object):

  def __init__(self):
    self.pub_x = rospy.Publisher('x_pid', Float64, queue_size=10)
    #self.buffer = tf2_ros.Buffer()
    self.listen_x = tf.TransformListener()
    #self.pose_sub = rospy.Subscriber('vicon/youbot/pose', PoseStamped, callback)

  def broadcaster(self):
    (translation, rotation) = self.listen_x.lookupTransform('/base_footprint', '/base_link', rospy.Time(0))
    rospy.loginfo("Successfully looked up transform!")
    self.pub_x.publish(translation[0])
    rospy.loginfo("Successfully published transform info to x_pid!")

if __name__ == '__main__':
  rospy.init_node('pose_node')
  P = PoseNode()
  rospy.loginfo("About to start the broadcaster loop!")
  while not rospy.is_shutdown():
    P.broadcaster()
