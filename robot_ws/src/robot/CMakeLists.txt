cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  message_generation
  move_base_msgs
  nav_core
  nav_msgs
  pluginlib
  roscpp
  roslib
  rospy
  sensor_msgs
  serial
  std_msgs
  tf
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

add_message_files(FILES supersonic.msg)

generate_messages(DEPENDENCIES std_msgs)

# ulimit -c unlimited
# ssh zhenyu@192.168.43.60
add_compile_options(-g -std=c++11)    #-g -> core dumped

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrt_star_planner
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    message_runtime
    move_base_msgs
    nav_core
    nav_msgs
    pluginlib
    roscpp
    roslib
    rospy
    sensor_msgs
    serial
    std_msgs
    tf
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

include_directories(
  include
  include/robot
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_node
  src/robot/robot.cpp
  src/robot/Quaternion_Solution.cpp
)
target_link_libraries(robot_node ${catkin_LIBRARIES})
add_dependencies(robot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(twist_transform src/robot/twist_transform.cpp)
target_link_libraries(twist_transform ${catkin_LIBRARIES})
