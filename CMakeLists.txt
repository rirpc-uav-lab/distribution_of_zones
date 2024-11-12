cmake_minimum_required(VERSION 3.8)
project(distribution_of_zones)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(uav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(COLCON_VARS ament_cmake nav_msgs rclcpp uav_msgs geometry_msgs std_msgs) # roslib

include_directories(include)

add_executable(distribution src/distribution.cpp)
# target_link_libraries(distribution )
ament_target_dependencies(distribution ${COLCON_VARS})

add_executable(distribution_test_client src/distribution_test_client.cpp)
# target_link_libraries(distribution_test_client )
ament_target_dependencies(distribution_test_client ${COLCON_VARS})

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

install(TARGETS
  distribution
  distribution_test_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
