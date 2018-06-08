import rospy
from std_msgs.msg import Float64


def callback(vicon):
  global setpoint
  pub_x.publish(vicon)
  #publish pose.x to state


def pose_node():
  rospy.init_node('pose_node')
  pub_x = rospy.Publisher('PID_X/state', Float64, queue_size=10)
  sub = rospy.Subscriber('vicon/youbot/pose', Float64, callback)
  #while not rospy.is_shutdown():
  #    pub_x.publish(vicon)
  #^is above snippet needed?


if __name__ == '__main__':
  pose_node()
