cmake_minimum_required(VERSION 3.8)
project(projectile_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(example_interfaces REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(my_node src/my_node.cpp)
add_executable(my_node_class src/my_node_class.cpp)
add_executable(pan_tilt_projectile src/pan_tilt_projectile.cpp)
ament_target_dependencies(my_node rclcpp example_interfaces)
ament_target_dependencies(my_node_class rclcpp example_interfaces)
ament_target_dependencies(
    pan_tilt_projectile
    rclcpp geometry_msgs std_msgs
  )

target_link_libraries(pan_tilt_projectile Eigen3::Eigen)

install(TARGETS
	my_node
  my_node_class
	pan_tilt_projectile
	DESTINATION lib/${PROJECT_NAME}
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
