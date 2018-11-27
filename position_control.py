#!/usr/bin/env python2.7

import numpy as np

import rospy
import tf
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64
from youbot_position.srv import PositionControl, PositionControlResponse

class Controller(object):
  '''A position control service node'''

  def __init__(self,
               global_frame,
               youbot_frame,
               setpoint_topic='setpoint_{}',
               control_topic='control_{}'):

    # Initialize constants
    self.stopping_distance = rospy.get_param('stopping_distance', 0.15)

    # Initialize system state
    self.velocity = Twist()
    self.frames = {'target': global_frame, 'source': youbot_frame}
    self.goal = np.zeros(2)
    self.pose = np.zeros(2)
    self.stopped = True

    # Setup publishers for setpoints, velocity, and PID enabling
    self.setpoint_pub = rospy.Publisher(setpoint_topic, Float64, queue_size=5, latch=True)

    self.velocity_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.pid_enable_pub = rospy.Publisher('pid_enable', Bool, queue_size=10)

    # Setup transform listener for pose transform
    self.tf_listener = tf.TransformListener()

    rospy.loginfo('Waiting for transform from {} to {}...'.format(global_frame, youbot_frame))
    self.tf_listener.waitForTransform(self.frames['target'], self.frames['source'], rospy.Time(), rospy.Duration(15))

    # Setup subscribers for control effort
    self.control_sub = rospy.Subscriber(control_topic, Float64, self.control_callback)

    # Start with PID disabled
    self.disable_control()

    # Setup timer to disable control when the setpoint is reached
    self.prox_timer = rospy.Timer(rospy.Duration(0.2), self.pose_callback)

    # Initialize service that will accept requested positions and set PID setpoints to match
    self.control_service = rospy.Service('position_control', PositionControl,
                                         self.position_control_service)
    rospy.loginfo("Position control service running!")

  # Executes when service is called; sets set_velocity to True
  def position_control_service(self, req):
    '''Callback receiving control service requests'''
    if req.stop:
      self.disable_control()
      self.velocity_pub.publish(Twist())
      return PositionControlResponse()

    rospy.logdebug("Received request: %s", req)
    self.goal = np.array([req.x, req.y])
    self.setpoint_pub.publish(req.x)
    self.stopped = False
    self.pid_enable_pub.publish(True)
    return PositionControlResponse()

  def disable_control(self):
    '''Disable the PID controllers'''
    self.stopped = True
    self.pid_enable_pub.publish(False)

  def control_callback(self, dimension, control):
    '''Callback receiving PID control output'''
    if self.stopped:
      return

    if dimension == 'x':
      self.velocity.linear.x = control.data
    elif dimension == 'y':
      self.velocity.linear.y = control.data

    self.fresh_val[dimension] = True
    if self.fresh_val['x'] and self.fresh_val['y']:
      self.velocity_pub.publish(self.velocity)
      self.fresh_val['x'] = False
      self.fresh_val['y'] = False

  def pose_callback(self, _):
    '''Handle stopping if the current pose is close enough to the goal'''
    (self.pose[0], self.pose[1], _), _ = self.tf_listener.lookupTransform(self.frames['target'],
                                                              self.frames['source'], rospy.Time())
    diff = self.pose - self.goal
    dist = diff.dot(diff)
    rospy.logdebug('Got distance: %s', dist)
    if dist <= self.stopping_distance:
      self.disable_control()
      self.velocity_pub.publish(Twist())


# Spin on control_service so that if the service goes down, we stop. This is better than doing it on
# rospy itself because it's more specific (per
# http://wiki.ros.org/rospy/Overview/Services#A.28Waiting_for.29_shutdown)
if __name__ == "__main__":
  rospy.init_node('position_control')
  GLOBAL_FRAME = rospy.get_param('~global_frame', 'world')
  YOUBOT_FRAME = rospy.get_param('~youbot_frame', 'base_link')
  C = Controller(GLOBAL_FRAME, YOUBOT_FRAME)
  C.control_service.spin()
