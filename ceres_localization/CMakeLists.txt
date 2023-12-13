cmake_minimum_required(VERSION 3.8)
project(ceres_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_localization REQUIRED)

install(DIRECTORY launch cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(robot_localization)


ament_package()
