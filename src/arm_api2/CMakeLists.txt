cmake_minimum_required(VERSION 3.8)
project(arm_api2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros REQUIRED)
find_package(joy REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(arm_api2_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(yaml-cpp REQUIRED)

# include headers
include_directories(include)

add_executable(moveit2_iface src/moveit2_iface.cpp src/moveit2_iface_node.cpp src/utils.cpp)
target_include_directories(moveit2_iface 
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE
  include/)

# TODO: Build moveit2_iface
target_compile_features(moveit2_iface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit2_iface moveit_ros moveit_ros_planning_interface moveit_visual_tools rclcpp arm_api2_msgs moveit_servo sensor_msgs geometry_msgs
)
target_link_libraries(moveit2_iface yaml-cpp)

# Install launch directory
install(DIRECTORY 
  launch
  config
  utils
  DESTINATION share/${PROJECT_NAME}
)

# joy stuff
install(TARGETS 
  moveit2_iface
  DESTINATION lib/${PROJECT_NAME})

# Install C++ files
add_executable(joy_ctl src/arm_joy_node.cpp src/arm_joy.cpp)

target_include_directories(joy_ctl 
  PRIVATE 
  include/
)

# python dependencies 
ament_target_dependencies(joy_ctl
  rclcpp std_msgs geometry_msgs std_srvs sensor_msgs control_msgs rclcpp_action)

install(TARGETS 
joy_ctl
DESTINATION lib/${PROJECT_NAME})

add_executable(keyboard_ctl src/servo_keyboard_input.cpp)

ament_target_dependencies(keyboard_ctl
  rclcpp std_msgs geometry_msgs std_srvs sensor_msgs control_msgs rclcpp_action)

install(TARGETS
  keyboard_ctl
  DESTINATION lib/${PROJECT_NAME})

set(SERVO_WATCHDOG_EXECUTABLE
  ${CMAKE_CURRENT_BINARY_DIR}/scripts/servo_watchdog.py)
file(COPY
  examples/scripts/servo_watchdog.py
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/scripts
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  examples/scripts/joint_sender_action_client_async.py
  examples/scripts/joint_sender_action_client.py
  examples/scripts/pose_sender_action_client_async.py
  examples/scripts/pose_sender_action_client.py
  examples/scripts/trajectory_sender_action_client_async.py
  examples/scripts/trajectory_sender_action_client.py
  examples/scripts/servo_twist_sender.py
  ${SERVO_WATCHDOG_EXECUTABLE}
  examples/scripts/circular_trajectory_publisher.py
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
