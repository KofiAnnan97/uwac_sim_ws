cmake_minimum_required(VERSION 3.1.1)
project(ekg_auv_testing)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   sensor_msgs
   geometry_msgs
   message_generation
   #seatrac_pkg
)

add_message_files(
   FILES
   USBLRequestSim.msg
   USBLResponseSim.msg
   VehicleLog.msg
   VehiclePose.msg
#   VehiclePoses.msg
   Packet.msg 
   Payload.msg
)

#include_directories(include ${catkin_INCLUDE_DIRS})
#add_library(
#   seatrac_pkg
#   src/MessagesTypes.py 
#   src/CommHandler.py 
#   src/CsvLogger.py
#)
#target_link_libraries(seatrac_pkg ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs 
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

#install(
#  TARGETS seatrac_pkg
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)
#install(
#  DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
