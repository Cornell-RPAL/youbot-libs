#!/usr/bin/env python2.7

import rospy
import tf
from std_msgs.msg import Float64, String

class PoseNode(object):

  def __init__(self, transform_target='world', transform_source='vicon/Mufasa_Base/Mufasa_Base'):
    self.transform_target = transform_target
    self.transform_source = transform_source
    self.pub_x = rospy.Publisher('x_pid', Float64, queue_size=10)
    self.pub_y = rospy.Publisher('y_pid', Float64, queue_size=10)
    self.pub_pos = rospy.Publisher('pos', String, queue_size=10)
    self.listen = tf.TransformListener()

  def broadcaster(self):
    try:
      (self._pos, rotation) = self.listen.lookupTransform(self.transform_target,
                                                            self.transform_source, rospy.Time(0))
      rospy.loginfo("Successfully looked up transform!")
      self.pub_x.publish(self._pos[0])
      self.pub_y.publish(self._pos[1])
      self.pub_pos.publish(str(self._pos))
      rospy.loginfo("Successfully published transform info to both PIDS!")
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      rospy.logerror("Failed to lookup transform between {} and {}! Got error: {}".format(
          self.transform_source, self.transform_target, e))

  def wait(self):
    rospy.loginfo('Waiting for transform...')
    self.listen.waitForTransform(self.transform_target, self.transform_source, rospy.Time(),
                                 rospy.Duration(5))


if __name__ == '__main__':
  rospy.init_node('pose_node')
  P = PoseNode()
  P.wait()
  while not rospy.is_shutdown():
    P.broadcaster()
