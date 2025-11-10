cmake_minimum_required(VERSION 3.8)
project(src_setup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(vertical_motion_system SHARED src/vertical_motion_system.cc)
target_compile_features(vertical_motion_system PRIVATE cxx_std_17)

target_link_libraries(vertical_motion_system
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

set_target_properties(vertical_motion_system PROPERTIES OUTPUT_NAME vertical_motion_system)

install(TARGETS vertical_motion_system LIBRARY DESTINATION lib)

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
