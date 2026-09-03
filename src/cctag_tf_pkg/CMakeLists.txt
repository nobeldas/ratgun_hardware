cmake_minimum_required(VERSION 3.8)
project(cctag_tf_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CCTag_DIR
  "/home/sunrise/ratgun_hardware/third_party/install/CCTag/lib/cmake/CCTag"
  CACHE PATH "Path to AliceVision CCTagConfig.cmake")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(CCTag CONFIG REQUIRED)

add_executable(cctag_tf_node src/cctag_tf_node.cpp)
ament_target_dependencies(cctag_tf_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  cv_bridge
)
target_link_libraries(cctag_tf_node
  CCTag::CCTag
  ${OpenCV_LIBRARIES}
)

set_target_properties(cctag_tf_node PROPERTIES
  INSTALL_RPATH "/home/sunrise/ratgun_hardware/third_party/install/CCTag/lib"
)

install(TARGETS cctag_tf_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
