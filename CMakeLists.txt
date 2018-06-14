cmake_minimum_required(VERSION 2.8.3)
project(youbot_position)
find_package(catkin REQUIRED COMPONENTS message_generation rospy std_msgs geometry_msgs)
add_service_files(
  DIRECTORY srv 
  FILES PositionControl.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
