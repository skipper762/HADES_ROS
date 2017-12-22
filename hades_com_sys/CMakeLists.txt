cmake_minimum_required(VERSION 2.8.3)
project(hades_com_sys)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

 add_message_files(
    FILES
    data_packet.msg
    audio_packet.msg
    sensor_packet.msg
    control_packet.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(COM_OP_NODE src/COMOP.cpp)
target_link_libraries(COM_OP_NODE ${catkin_LIBRARIES})
add_dependencies(COM_OP_NODE HADES_generate_messages_cpp)

find_package(message_generation)
catkin_package(CATKIN_DEPENDS message_runtime)

