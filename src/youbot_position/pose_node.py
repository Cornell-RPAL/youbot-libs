#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, PoseStamped

class PoseNode(object):
    #left the TODOs up because they were very helpful; will remove once we're done

    def __init__(self):
        self.pub_x = rospy.Publisher('x_pid', Float64, queue_size=10)
        self.pose_sub = rospy.Subscriber('vicon/youbot/pose', PoseStamped, callback)

    def callback(self, vicon):
      # TODO: pub_x isn't in scope here. You should either make it global, make this into a class-style
      # node and make pub_x a field (probably the best approach), or you could curry `callback` with
      # `pub_x` (this is not the best approach; I mention it mostly for completeness)
      # TODO: You probably want to publish vicon.pose.position.x (for the X coordinate; when we get to
      # publishing both coordinates we'll talk about how to pick which coordinate to send). See
      # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseStamped.html for the docs on the
      # message type vicon/youbot/pose uses
      self.pub_x.publish(vicon.pose.position.x)
      # publish pose.x to state

      # TODO: The vicon_bridge node uses geometry_msgs/PoseStamped as its message type for the pose
      # topics, so you should use that instead of Float64 here. It might be worthwhile to spend some
      # time playing around with rostopic info, rostopic echo, and rostopic list with the vicon_bridge
      # to get a sense of the message types used?



if __name__ == '__main__':
  rospy.init_node('pose_node')
  P = PoseNode()
  rospy.spin()
