#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class Control(data):
    def __init__(self):
        self.setpoint = 0
        self.request_count = 0

    def one_request():
        if #probably obvious but how to count number of service calls?
            return true;
        return false;

def handle_position_control(req):
    print "Point = %"%(req)
    pub_setpoint_x.publish(req.req)
    #publish req.req to setpoint_x

def callback(data):
    if one_request
        vel = Twist()
        vel.linear = data
        vel.angular = 0 #since for now we're working in one dimension?
        pub_vel.publish(vel)
    #get control_x (I think this is x_velocity? Sorry for bad naming)
    #Also, once we get ^above value, what do we do with it?

def position_control_server():
    rospy.init_node('position_control')
    s = rospy.Service('position_control', PositionControl, handle_position_control)
    pub_setpoint_x = rospy.Publisher('setpoint_x', Float64, queue_size=10)
    pub_vel = rospy.Publisher('x_velocity', Float64, queue_size = 10)
    sub_control_x = rospy.Subscriber('control_x', Float64, callback)
    rospy.spin()

if __name__ == "__main__":
    position_control_server()
