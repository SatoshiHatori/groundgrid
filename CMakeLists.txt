cmake_minimum_required(VERSION 3.16)
project(groundgrid)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(grid_map_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NextCloud.srv"
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib SHARED
  src/GroundGrid.cpp
  src/GroundSegmentation.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp geometry_msgs sensor_msgs std_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs
  grid_map_core grid_map_ros grid_map_cv grid_map_msgs
  PCL pcl_conversions cv_bridge image_transport
)

add_library(${PROJECT_NAME}_component SHARED
  src/GroundGridCp.cpp
)
target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp rclcpp_components geometry_msgs sensor_msgs std_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs
  grid_map_core grid_map_ros grid_map_cv grid_map_msgs
  PCL pcl_conversions cv_bridge image_transport
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "groundgrid::GroundGridCp")

add_executable(${PROJECT_NAME}_node src/GroundGridNode.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_component)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp rclcpp_components geometry_msgs sensor_msgs std_msgs nav_msgs
  tf2 tf2_ros tf2_geometry_msgs
  grid_map_core grid_map_ros grid_map_cv grid_map_msgs
  PCL pcl_conversions cv_bridge image_transport
)

install(TARGETS
  ${PROJECT_NAME}_lib
  ${PROJECT_NAME}_component
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()

