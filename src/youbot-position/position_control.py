#!/usr/bin/env python
import rospy

def handle_position_control(req):
    print "Point = %"%(req)
    #return cmd_vel

def callback(data):
    #get control_x
    
def position_control_server():
    rospy.init_node('position_control')
    s = rospy.Service('position_control', PositionControl, handle_position_control)
    pub_setpoint_x = rospy.Publisher('setpoint_x', Float64, queue_size=10)
    sub_control_x = rospy.Subscriber('control_x', Float64, callback)
    rospy.spin()

if __name__ == "__main__":
    position_control_server()
