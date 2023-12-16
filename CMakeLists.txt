cmake_minimum_required(VERSION 3.0.2)
project(kvaser_can_rospkg)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  can_msgs
  message_generation
  dynamic_reconfigure
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  FramePlus.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  WriteService.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)


# ##################################
# # catkin specific configuration ##
# ##################################
catkin_package(
  CATKIN_DEPENDS
  dynamic_reconfigure
)

# ##########
# # Build ##
# ##########

# Specify additional locations of header files
# Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

# Declare a C++ library
add_library(kvasercan_interface
  include/${PROJECT_NAME}/kvasercan_interface.hpp
  src/kvasercan_interface.cpp
)
target_link_libraries(kvasercan_interface canlib)

# kvasercan_bridge
add_executable(kvasercan_bridge
  include/${PROJECT_NAME}/kvasercan_bridge.hpp
  src/kvasercan_bridge.cpp
  src/kvasercan_bridge_node.cpp
)
target_link_libraries(kvasercan_bridge
  kvasercan_interface
  ${catkin_LIBRARIES}
)

# kvasercan_listchannel_node
add_executable(kvasercan_listchannel
  src/kvasercan_listchannel_node.cpp
)
target_link_libraries(kvasercan_listchannel
  kvasercan_interface
  ${catkin_LIBRARIES}
)


# ############
# # Testing ##
# ############

add_executable(vcan_write_test
  test/vcanfd_write_test.cpp
)
target_link_libraries(vcan_write_test
  ${catkin_LIBRARIES}
  canlib
)

add_executable(vcan_monitor_test
  test/vcanfd_monitor_test.cpp
)
target_link_libraries(vcan_monitor_test
  ${catkin_LIBRARIES}
  canlib
)

add_executable(service_test
  test/service_test.cpp
)
target_link_libraries(service_test
  ${catkin_LIBRARIES}
)

# ############
# # Install ##
# ############

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

