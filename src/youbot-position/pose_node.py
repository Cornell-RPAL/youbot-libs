#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float64


def callback(vicon):
  # TODO: Setpoint is never actually used or initialized. I don't think you need it in this node?
  global setpoint
  # TODO: pub_x isn't in scope here. You should either make it global, make this into a class-style
  # node and make pub_x a field (probably the best approach), or you could curry `callback` with
  # `pub_x` (this is not the best approach; I mention it mostly for completeness)
  # TODO: You probably want to publish vicon.pose.position.x (for the X coordinate; when we get to
  # publishing both coordinates we'll talk about how to pick which coordinate to send). See
  # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html for the docs on the
  # message type vicon/youbot/pose uses
  pub_x.publish(vicon)
  # publish pose.x to state


def pose_node():
  rospy.init_node('pose_node')
  pub_x = rospy.Publisher('PID_X/state', Float64, queue_size=10)
  # TODO: The vicon_bridge node uses geometry_msgs/PoseStamped as its message type for the pose
  # topics, so you should use that instead of Float64 here. It might be worthwhile to spend some
  # time playing around with rostopic info, rostopic echo, and rostopic list with the vicon_bridge
  # to get a sense of the message types used?
  sub = rospy.Subscriber('vicon/youbot/pose', Float64, callback)
  #while not rospy.is_shutdown():
  #    pub_x.publish(vicon)
  #^is above snippet needed?
  # TODO: No, all the publishing happens in `callback`, so you have nothing to publish here. What
  # you do need, however, is still some sort of wait to ensure that the node doesn't exit before
  # you're done with it. ROS has a lot of ways to do this, but here, because you don't need to do
  # any more work in this function after this point, I would use the following:
  rospy.spin()


if __name__ == '__main__':
  pose_node()
