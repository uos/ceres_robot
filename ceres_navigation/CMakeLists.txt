cmake_minimum_required(VERSION 3.8)
project(ceres_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ceres_localization REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_util REQUIRED)
find_package(navigation2 REQUIRED)

add_executable(bt_simple src/bt_simple.cpp)
ament_target_dependencies(bt_simple 
  rclcpp 
  nav2_behavior_tree 
  nav2_util
)

install(TARGETS
  bt_simple
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ceres_localization)


ament_package()
