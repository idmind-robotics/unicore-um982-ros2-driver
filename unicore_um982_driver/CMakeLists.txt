cmake_minimum_required(VERSION 3.8)
project(unicore_um982_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# PVTSLN parser library, split out so it can be unit tested without a ROS node
add_library(pvtsln_parser src/pvtsln_parser.cpp)
target_include_directories(pvtsln_parser PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Add executable
add_executable(unicore_um982_driver_node
  src/unicore_um982_driver_node.cpp
)
ament_target_dependencies(unicore_um982_driver_node rclcpp serial_driver sensor_msgs geometry_msgs diagnostic_updater)
target_link_libraries(unicore_um982_driver_node pvtsln_parser)

# Install executable
install(TARGETS
  unicore_um982_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config/
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

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pvtsln_parser test/test_pvtsln_parser.cpp)
  target_link_libraries(test_pvtsln_parser pvtsln_parser)
endif()

ament_package()
