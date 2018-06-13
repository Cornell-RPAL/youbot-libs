#!/usr/bin/env python2.7

import rospy
import tf
from std_msgs.msg import Float64


class PoseNode(object):

  def __init__(self, transform_target='world', transform_source='base_link'):
    self.transform_target = transform_target
    self.transform_source = transform_source
    self.pub_x = rospy.Publisher('x_pid', Float64, queue_size=10)
    self.listen_x = tf.TransformListener()

  def broadcaster(self):
    try:
      (translation, rotation) = self.listen_x.lookupTransform(self.transform_target,
                                                              self.transform_source, rospy.Time(0))
      rospy.loginfo("Successfully looked up transform!")
      self.pub_x.publish(translation[0])
      rospy.loginfo("Successfully published transform info to x_pid!")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      rospy.logerror(
          "Failed to lookup transform between base_link and map! Got error: {}".format(e))

  def wait(self):
    rospy.loginfo('Waiting for transform...')
    self.listen_x.waitForTransform(self.transform_target, self.transform_source, rospy.Time(),
                                   rospy.Duration(5))


if __name__ == '__main__':
  rospy.init_node('pose_node')
  P = PoseNode()
  P.wait()
  while not rospy.is_shutdown():
    P.broadcaster()
