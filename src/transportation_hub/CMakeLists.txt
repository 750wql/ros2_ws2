cmake_minimum_required(VERSION 3.8)
project(transportation_hub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(transportation_msgs REQUIRED)
find_package(judger_interfaces REQUIRED)
add_executable(transportation_hub_node src/transportation_hub_node.cpp)
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
ament_target_dependencies(transportation_hub_node
  rclcpp
  std_msgs
  transportation_msgs
        judger_interfaces
)
install(TARGETS
  transportation_hub_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
