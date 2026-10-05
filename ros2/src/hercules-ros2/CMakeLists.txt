cmake_minimum_required(VERSION 3.8)
project(hercules-ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(airsim_interfaces REQUIRED)
# Link only the modules we use (not ${OpenCV_LIBS}, which lists every module once
# cv_bridge has found OpenCV): opencv_viz pulls in VTK, which fails to generate
# alongside PCL on Ubuntu 26.04 (Lyrical).
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

# find_package(airsim_ros_pkgs REQUIRED)

# TO BUILD WITH CUSTOM MESSAGES
# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/YourCustomMessage.msg"
#   DEPENDENCIES std_msgs geometry_msgs
# )

# ament_export_dependencies(rosidl_default_runtime)

# EXAMPLE OF HOW YOU WOULD CREATE NEW CPP ROS2 EXECUTABLE
# add_executable(airsim_node src/airsim_node.cpp)
# target_link_libraries(airsim_node airsim_ros AirLib)
# target_link_libraries(airsim_node
#   rclcpp::rclcpp
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(imu_linear_interpolator_node src/imu_linear_interpolator.cpp)
target_link_libraries(imu_linear_interpolator_node
  rclcpp::rclcpp
  Eigen3::Eigen
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
)

install(TARGETS
  imu_linear_interpolator_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(imu_linear_interpolator_v2_node src/imu_linear_interpolator_v2.cpp)
target_link_libraries(imu_linear_interpolator_v2_node
  rclcpp::rclcpp
  Eigen3::Eigen
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
  imu_linear_interpolator_v2_node
  DESTINATION lib/${PROJECT_NAME}
)


# depth viewing node______________
add_executable(view_depthperspective_pointcloud_node
 src/view_depthperspective_pointcloud.cpp)
target_link_libraries(view_depthperspective_pointcloud_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
  cv_bridge::cv_bridge
  message_filters::message_filters
)
if(message_filters_VERSION VERSION_LESS 5.0.0)
  # Jazzy and older
  target_compile_definitions(view_depthperspective_pointcloud_node
    PRIVATE USE_OLD_RMW_QOS_MESSAGE_FILTERS=1)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
  view_depthperspective_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

# thermal image node___________________
add_executable(thermal_image_node
 src/thermal_image_node.cpp)
target_link_libraries(thermal_image_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
  cv_bridge::cv_bridge
)

# thermal image segmentation based node___________________
add_executable(thermal_image_segmentation_based_node
 src/thermal_image_segmentation_based_node.cpp)
target_link_libraries(thermal_image_segmentation_based_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
  cv_bridge::cv_bridge
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
thermal_image_node
thermal_image_segmentation_based_node
  DESTINATION lib/${PROJECT_NAME}
)
# ____________________________
# bag timestamp checker node______
add_executable(bag_timestamp_checker_node
 src/bag_timestamp_checker.cpp)
 target_link_libraries(bag_timestamp_checker_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
  cv_bridge::cv_bridge
  ${airsim_interfaces_TARGETS}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
bag_timestamp_checker_node
  DESTINATION lib/${PROJECT_NAME}
)
# ________________________________
# topic time sync checker node__________________
add_executable(topic_time_sync_checker_node
 src/topic_time_sync_checker.cpp)
 target_link_libraries(topic_time_sync_checker_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
  cv_bridge::cv_bridge
  ${airsim_interfaces_TARGETS}
)


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
topic_time_sync_checker_node
  DESTINATION lib/${PROJECT_NAME}
)
#_______________________________________________
# imu republisher node_________________________
add_executable(imu_republisher_node
 src/imu_republisher_node.cpp)
 target_link_libraries(imu_republisher_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
  cv_bridge::cv_bridge
  ${airsim_interfaces_TARGETS}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
imu_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)
# _____________________________________________
# downsample and synchronize topics node_______
add_executable(downsample_synchronizer_node
 src/downsample_synchronizer_node.cpp)
 target_link_libraries(downsample_synchronizer_node
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_sensor_msgs::tf2_sensor_msgs
  cv_bridge::cv_bridge
  ${airsim_interfaces_TARGETS}
  opencv_core
  opencv_imgproc
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


install(TARGETS
downsample_synchronizer_node
  DESTINATION lib/${PROJECT_NAME}
)

# _____________________________________________

# Install Python scripts
install(PROGRAMS
  scripts/test_depthperspective.py 
  scripts/depthvis.py 
  scripts/multi_robot_topic_monitor.py 
  scripts/full_topic_echo.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
