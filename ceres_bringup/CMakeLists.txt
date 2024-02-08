cmake_minimum_required(VERSION 3.8)
project(ceres_bringup)


# Configure C++ compiler
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
ament_export_dependencies(xacro)
ament_export_dependencies(urdf)

ament_package()