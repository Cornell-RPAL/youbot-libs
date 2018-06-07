import rospy
from std_msgs.msg import Float64

def callback(vicon_data):
    global setpoint
    #get vicon_data

def pose_node():
    rospy.init_node('pose_node')
    pub_x = rospy.Publisher('pose_node_x', Float64, queue_size=10)
    #where does publishing to state topic happen?
    sub = rospy.Subscriber('vicon_data', Float64, callback)
    while not rospy.is_shutdown():
        pub_x.publish(vicon_data_x)

if __name__ == '__main__':
        pose_node()
