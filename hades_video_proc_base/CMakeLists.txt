cmake_minimum_required(VERSION 2.8.3)
project(hades_video_proc_base)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  hades_base_stn
)

 generate_messages(
  DEPENDENCIES
   std_msgs
 )

#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hades_video_proc_base
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
#)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  hades_base_stn
)

add_executable(HADES_VID_PROC_NODE_BASE src/hades_base_stn_proc.cpp)
target_link_libraries(HADES_VID_PROC_NODE_BASE ${catkin_LIBRARIES})
add_dependencies(HADES_VID_PROC_NODE_BASE HADES_generate_messages_cpp)

find_package(message_generation)
catkin_package(CATKIN_DEPENDS message_runtime)

