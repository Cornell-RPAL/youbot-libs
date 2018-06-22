#!/usr/bin/env python2.7

import rospy
import message_filters
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from youbot_position.srv import PositionControl, PositionControlResponse

STOP_DIST = 0.1

class Controller(object):
  '''A position control service node'''

  def __init__(self):
    # setpoint_topic='setpoint_x', control_topic='control_x'):
    # set called to False initially, and iniialize publishers that publish
    # intended setpoint and the desired velocity for the youBot.
    self.set_velocity = False
    self.fresh_x = False
    self.fresh_y = False
    self.setpoint_pub_x = rospy.Publisher('setpoint_x', Float64, queue_size=5, latch=True)
    self.setpoint_pub_y = rospy.Publisher('setpoint_y', Float64, queue_size=5, latch=True)
    self.velocity_pub = rospy.Publisher('mufasa/cmd_vel', Twist, queue_size=10)
    self.velocity = Twist()
    # Initialize subscriber that listens to output of PIDs
    self.control_sub_x = message_filters.Subscriber('control_x', Float64)
    self.control_sub_y = message_filters.Subscriber('control_y', Float64)
    #Initialize subscriber that listens to position
    self.pose_sub_x = message_filters.Subscriber('x_pid', Float64)
    self.pose_sub_y = message_filters.Subscriber('y_pid', Float64)
    ts_x = message_filters.ApproximateTimeSynchronizer([self.control_sub_x, self.pose_sub_x], 10, 0.1, allow_headerless=True)
    ts_x.registerCallback(self.control_callback_x)
    ts_y = message_filters.ApproximateTimeSynchronizer([self.control_sub_y, self.pose_sub_y], 10, 0.1, allow_headerless=True)
    ts_y.registerCallback(self.control_callback_y)
    # Initialize service that will accept requested positions and set PID setpoints to match
    self.control_service = rospy.Service('position_control', PositionControl,
                                         self.position_control_service)
    rospy.loginfo("Position control service running!")

  # Executes when service is called; sets set_velocity to True
  def position_control_service(self, req):
    '''Callback receiving control service requests'''
    if req.stop:
      null_vector = Twist()
      self.velocity_pub.publish(null_vector)
      self.set_velocity = False
      return PositionControlResponse()

    rospy.loginfo("Received request: %s", req)
    self.setpoint_pub_x.publish(req.x)
    self.setpoint_pub_y.publish(req.y)
    self.set_velocity = True
    return PositionControlResponse()

  # Check if service has been called. If so, while listening to control_topic,
  # set our linear velocity to the data and publish our current velocity.
  def control_callback_x(self, control_x, x_pid, req):
    '''Callback receiving x PID control output'''
    if self.set_velocity:
      self.velocity.linear.x = -control_x.data
      self.fresh_x = True
      if self.fresh_y:
        self.velocity_pub.publish(self.velocity)
        self.fresh_x = False
      if abs(req.x - x_pid.data) <= STOP_DIST:
        self.velocity.linear.x = 0.0
        self.velocity_pub.publish(self.velocity)
        self.fresh_x = True


  def control_callback_y(self, control_y, y_pid, req):
    '''Callback receiving y PID control output'''
    if self.set_velocity:
      self.velocity.linear.y = -control_y.data
      self.fresh_y = True
      if self.fresh_x:
        self.velocity_pub.publish(self.velocity)
        self.fresh_y = False
      if abs(req.y - y_pid.data) <= STOP_DIST:
        self.velocity.linear.y = 0.0
        self.velocity_pub.publish(self.velocity)
        self.fresh_y = True

# Spin on control_service so that if the service goes down, we stop. This is better than doing it on
# rospy itself because it's more specific (per
# http://wiki.ros.org/rospy/Overview/Services#A.28Waiting_for.29_shutdown)
if __name__ == "__main__":
  rospy.init_node('position_control')
  C = Controller()
  C.control_service.spin()
