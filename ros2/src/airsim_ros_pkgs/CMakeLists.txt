cmake_minimum_required(VERSION 3.5)
project(airsim_ros_pkgs)

set(CMAKE_BUILD_TYPE Debug)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(airsim_interfaces REQUIRED)
# Link only the modules we use (not ${OpenCV_LIBS}, which lists every module once
# cv_bridge has found OpenCV): opencv_viz pulls in VTK, which fails to generate
# alongside PCL on Ubuntu 26.04 (Lyrical).
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

# Newer pcl_conversions (e.g. Lyrical) exports a CMake target; Jazzy's only
# exposes plain variables, so wrap them in the same target name.
if(NOT TARGET pcl_conversions::pcl_conversions)
  add_library(pcl_conversions::pcl_conversions INTERFACE IMPORTED)
  set_target_properties(pcl_conversions::pcl_conversions PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${pcl_conversions_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${pcl_conversions_LIBRARIES}")
endif()

set(AIRSIM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../../../)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel")
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc
-fmax-errors=10")
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.1/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

set(INCLUDE_DIRS include 
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  )
include_directories(${INCLUDE_DIRS})

# hercules settings parser____
add_library(hercules_settings_parser src/hercules_settings_parser.cpp)
target_link_libraries(hercules_settings_parser AirLib)
# ____________________________

add_library(airsim_settings_parser src/airsim_settings_parser.cpp)
target_link_libraries(airsim_settings_parser AirLib)

add_library(pd_position_controller_simple src/pd_position_controller_simple.cpp)
target_link_libraries(pd_position_controller_simple AirLib)
target_link_libraries(pd_position_controller_simple
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${airsim_interfaces_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

# hercules ros wrapper___________
add_library(hercules_ros src/hercules_ros_wrapper.cpp)
target_link_libraries(hercules_ros opencv_core opencv_imgproc yaml-cpp AirLib hercules_settings_parser)
target_include_directories(hercules_ros PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(hercules_ros
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  image_transport::image_transport
  tf2_ros::tf2_ros
  cv_bridge::cv_bridge
  ${airsim_interfaces_TARGETS}
  ${mavros_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_sensor_msgs::tf2_sensor_msgs
  tf2::tf2
  pcl_conversions::pcl_conversions
  ${PCL_LIBRARIES}
)
# _______________________________


add_library(airsim_ros src/airsim_ros_wrapper.cpp)
target_link_libraries(airsim_ros opencv_core opencv_imgproc yaml-cpp AirLib airsim_settings_parser)
target_include_directories(airsim_ros PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(airsim_ros
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  image_transport::image_transport
  tf2_ros::tf2_ros
  cv_bridge::cv_bridge
  ${airsim_interfaces_TARGETS}
  ${mavros_msgs_TARGETS}
  ${std_srvs_TARGETS}
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_sensor_msgs::tf2_sensor_msgs
  tf2::tf2
  pcl_conversions::pcl_conversions
  ${PCL_LIBRARIES}
)

# hercules node______
add_executable(hercules_node src/airsim_node_hercules.cpp)
target_link_libraries(hercules_node hercules_ros AirLib)
target_link_libraries(hercules_node
  rclcpp::rclcpp
)
# _______________________________

# hercules SYNCED node (atomic / lockstep, DepthPlanar, shared-stamp) ______
# Self-contained: drives AirLib directly, re-derives the same topic/TF interface as
# hercules_node so hercules_nav_bridge works unchanged. Does NOT use hercules_ros.
add_executable(hercules_synced_node src/airsim_node_hercules_synced.cpp)
target_link_libraries(hercules_synced_node AirLib)
target_link_libraries(hercules_synced_node
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${rosgraph_msgs_TARGETS}
)
# _______________________________


add_executable(airsim_node src/airsim_node.cpp)
target_link_libraries(airsim_node airsim_ros AirLib)
target_link_libraries(airsim_node
  rclcpp::rclcpp
)

add_executable(pd_position_controller_simple_node
  src/pd_position_controller_simple_node.cpp)
target_link_libraries(pd_position_controller_simple_node
  pd_position_controller_simple airsim_ros hercules_ros AirLib)
target_link_libraries(pd_position_controller_simple_node
  rclcpp::rclcpp
  ${mavros_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

# rosidl_target_interfaces(airsim_node
# ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rosidl_target_interfaces(pd_position_controller_simple_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS airsim_node hercules_node hercules_synced_node pd_position_controller_simple_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS airsim_ros hercules_ros pd_position_controller_simple
  airsim_settings_parser hercules_settings_parser
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/test_rviz2_markers.py 
  scripts/uav_keyboard_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_dependencies(rclcpp)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(geographic_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_sensor_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(tf2)
ament_export_dependencies(image_transport)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(mavros_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(airsim_interfaces)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(PCL)

ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(airsim_settings_parser hercules_settings_parser pd_position_controller_simple
  airsim_ros hercules_ros)

ament_package()
