cmake_minimum_required(VERSION 3.8)
project(src_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

install(
  DIRECTORY config description launch models worlds
  DESTINATION share/${PROJECT_NAME}
)
add_executable(multi_moving_box_controller_gz src/multi_moving_box_controller_rosgz.cpp)
ament_target_dependencies(multi_moving_box_controller_gz
  rclcpp geometry_msgs ros_gz_interfaces)
  
install(TARGETS multi_moving_box_controller_gz
  DESTINATION lib/${PROJECT_NAME})


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
