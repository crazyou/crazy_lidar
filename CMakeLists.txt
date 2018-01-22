cmake_minimum_required(VERSION 2.8.3)
project(crazylidar_node)

FILE(GLOB SELIDAR_DRV_SRC 
  "./src/drv/*.cpp"
  "./src/utils/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  message_generation
  rospy
  std_msgs
  std_srvs
)

include_directories(
  ./src/utils
  ./src/drv
  ${catkin_INCLUDE_DIRS}
)


generate_messages(
  DEPENDENCIES
  std_msgs
)


catkin_package()
add_executable(crazylidarNode src/node.cpp ${SELIDAR_DRV_SRC})
target_link_libraries(crazylidarNode ${catkin_LIBRARIES})

add_executable(motor_client src/motor_client.cpp ${SELIDAR_DRV_SRC})
target_link_libraries(motor_client ${catkin_LIBRARIES})

install(TARGETS crazylidarNode
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
