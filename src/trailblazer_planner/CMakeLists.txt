cmake_minimum_required(VERSION 3.8)
project(trailblazer_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# include cpp "include" directory
include_directories(include)



# install python modules
ament_python_install_package(${PROJECT_NAME})


# test python executables
install(PROGRAMS
  scripts/example_node_1_2.py
  scripts/service_example.py
  scripts/node_service_1_2.py
  scripts/service_planner_example.py
  scripts/service_planner_task3.py
  scripts/service_planner_task5.py
  scripts/service_planner_task7.py
  scripts/service_planner_task9.py
  DESTINATION lib/${PROJECT_NAME}
)

# install package directories
install(DIRECTORY
  launch
#  rviz
#  params
  DESTINATION share/${PROJECT_NAME}
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

# End of Code
