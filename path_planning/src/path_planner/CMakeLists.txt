cmake_minimum_required(VERSION 3.8)
project(path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(path_planner 
  src/path_planner.cpp
  src/shorthand.cpp
  src/utils.cpp
)
ament_target_dependencies(path_planner rclcpp std_msgs geometry_msgs nav_msgs visualization_msgs)

install(TARGETS
  path_planner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
