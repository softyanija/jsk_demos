cmake_minimum_required(VERSION 2.8.3)
project(multi_device_view)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  dynamic_reconfigure
 )

add_message_files(
  FILES
  Ellipse.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/MultiDeviceView.cfg
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_lecture
CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

install(DIRECTORY euslisp
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


