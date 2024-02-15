cmake_minimum_required(VERSION 3.1.1)
project(ekg_auv_testing)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   sensor_msgs
   geometry_msgs
   message_generation
)

add_message_files(
   FILES
   #bcn_frame.msg
   #bcn_frame_array.msg
   #bcn_pose.msg
   #bcn_pose_array.msg
   #bcn_status.msg
   #bcn_status_array.msg
   #bcn_remote_gps.msg
   #loc.msg
   #IverOSI.msg
   #head.msg
   USBLRequestSim.msg
   USBLResponseSim.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS 
   roscpp 
   rospy 
   std_msgs 
   sensor_msgs 
   geometry_msgs 
   message_runtime
)

include_directories(
# include
   ${catkin_INCLUDE_DIRS}
)

