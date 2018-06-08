#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from youbot_position.srv import PositionControl, PositionControlResponse


class Controller(object):
  '''A position control service node'''

  def __init__(self, setpoint_topic='setpoint_x', control_topic='control_x'):
    self.called = False
    self.setpoint_pub = rospy.Publisher(setpoint_topic, Float64, queue_size=5, latch=True)
    self.velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # NOTE: I'm storing the velocity as a field because that will make the restructuring easier when
    # the controller is responsible for both X and Y components of the velocity
    self.velocity = Twist()
    self.control_sub = rospy.Subscriber(control_topic, Float64, self.control_callback)
    self.control_service = rospy.Service('position_control', PositionControl,
                                         self.position_control_service)
    rospy.loginfo("Position control service running!")

  def position_control_service(self, req):
    '''Callback receiving control service requests'''
    rospy.loginfo("Received request: %s", req)
    self.setpoint_pub.publish(req.x)
    self.called = True
    return PositionControlResponse()

  def control_callback(self, control):
    '''Callback receiving PID control output'''
    if not self.called:
      return

    self.velocity.linear.x = control.data
    self.velocity_pub.publish(self.velocity)


if __name__ == "__main__":
  rospy.init_node('position_control')
  C = Controller()
  C.control_service.spin()
