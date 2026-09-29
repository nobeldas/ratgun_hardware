cmake_minimum_required(VERSION 3.10)
project(target_prediction_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(crlb_line_fitting src/crlb_line_fitting.cpp)
add_executable(kalman_prediction src/kalman_prediction.cpp)
ament_target_dependencies(
  crlb_line_fitting
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
)
ament_target_dependencies(
  kalman_prediction
  rclcpp
  geometry_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(crlb_line_fitting Eigen3::Eigen)
target_link_libraries(kalman_prediction Eigen3::Eigen)

install(
  TARGETS
    crlb_line_fitting
    kalman_prediction
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

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

ament_package()
