cmake_minimum_required(VERSION 3.8)
project(mars_rover)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(simulation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenCV 4 QUIET)
if (NOT OpenCV_FOUND)
  find_package(OpenCV REQUIRED)
endif ()
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# inlude cpp header
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# create executable for image subscriber
add_executable(image_subscriber src/ImageSubscriber.cpp)
ament_target_dependencies(image_subscriber
  OpenCV
  rclcpp
  sensor_msgs
  cv_bridge
)

# create executable for image process
add_executable(image_process src/ImageProcessNode.cpp)
ament_target_dependencies(image_process
  OpenCV
  rclcpp
  sensor_msgs
  cv_bridge
)

# create executable for image analyze
add_executable(image_analyze src/ImageAnalyzeNode.cpp)
ament_target_dependencies(image_analyze
  OpenCV
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(image_subscriber ${OpenCV_LIBRARIES})
target_link_libraries(image_process ${OpenCV_LIBRARIES})
target_link_libraries(image_analyze ${OpenCV_LIBRARIES})

# install executable
install(TARGETS
  image_subscriber
  image_process
  image_analyze
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  nodes/move_arm
  nodes/move_mast
  nodes/move_wheel
  nodes/run_demo
  nodes/image_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
