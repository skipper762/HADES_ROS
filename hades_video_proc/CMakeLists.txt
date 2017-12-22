cmake_minimum_required(VERSION 2.8.3)
project(hades_video_proc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	hades_com_sys
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

 generate_messages(
  DEPENDENCIES
   std_msgs
 )

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hades_video_proc
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
#)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  hades_com_sys
)

add_executable(HADES_VID_PROC_NODE src/hades_video_proc.cpp)
target_link_libraries(HADES_VID_PROC_NODE ${catkin_LIBRARIES})
add_dependencies(HADES_VID_PROC_NODE HADES_generate_messages_cpp)

find_package(message_generation)
catkin_package(CATKIN_DEPENDS message_runtime)


