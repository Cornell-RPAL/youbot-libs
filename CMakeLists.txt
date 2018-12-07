cmake_minimum_required(VERSION 2.8.3)
project(youbot_position)
find_package(catkin REQUIRED COMPONENTS message_generation rospy roscpp std_msgs geometry_msgs)
add_service_files(
  DIRECTORY srv
  FILES PositionControl.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime
)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#add_executable(Autotune autotune.cpp)
#target_link_libraries(Autotune ${catkin_LIBRARIES})
#add_dependencies(Autotune ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
