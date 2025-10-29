cmake_minimum_required(VERSION 3.8)
project(my_robot_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)
add_library(dijkstra_planner SHARED src/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner rclcpp nav_msgs geometry_msgs tf2_ros nav2_core nav2_msgs nav2_costmap_2d nav2_util pluginlib)

add_library(a_star SHARED src/a_star.cpp)
ament_target_dependencies(a_star rclcpp nav_msgs geometry_msgs tf2_ros nav2_core nav2_costmap_2d nav2_util pluginlib)
ament_python_install_package(${PROJECT_NAME})

pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)

install(PROGRAMS
  ${PROJECT_NAME}/dijkstra_planner.py
  ${PROJECT_NAME}/a_star.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)
install(TARGETS
  dijkstra_planner
  a_star
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES global_planner_plugins.xml
DESTINATION share/${PROJECT_NAME})

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

ament_export_include_directories(include)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2_ros nav2_core nav2_costmap_2d nav2_util pluginlib)
ament_package()
