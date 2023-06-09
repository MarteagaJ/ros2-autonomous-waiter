cmake_minimum_required(VERSION 3.8)
project(autonomous_waiter_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(cmd_vel_publisher publishers/cmd_vel_publisher.cpp)
ament_target_dependencies(cmd_vel_publisher rclcpp geometry_msgs)

#add_executable(odom_sub subscribers/odometry_twist_subscriber.cpp)
#ament_target_dependencies(odom_sub rclcpp geometry_msgs)

install(
  DIRECTORY config description launch publishers subscribers
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
 cmd_vel_publisher
#  odom_sub
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
