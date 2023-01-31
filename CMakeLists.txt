cmake_minimum_required(VERSION 3.5)
project(sly_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake_gtest)
find_package(lifecycle_msgs REQUIRED)


ament_add_gtest(controller_tests test/sly_controller_test.cpp src/sly_controller.cpp)
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(include/sly_controller)


add_executable(sly_controller_node src/sly_controller_node.cpp src/motor.cpp src/sly_controller.cpp)

add_executable(lifecycle_manager_node src/lifecyle_manager.cpp)

target_include_directories(sly_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  sly_controller_node
  "std_msgs"
  "nav_msgs"
  "sensor_msgs"
  "rclcpp"
  "rcutils"
  "geometry_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "Eigen3"
)

target_include_directories(lifecycle_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
  lifecycle_manager_node
  "lifecycle_msgs"
  "rclcpp"
  "rcutils"
)


install(TARGETS sly_controller_node lifecycle_manager_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
