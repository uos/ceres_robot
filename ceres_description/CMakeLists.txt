cmake_minimum_required(VERSION 3.8)
project(ceres_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
#find_package(sick_tim REQUIRED)
#find_package(velodyne_description REQUIRED)

install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(xacro)
ament_export_dependencies(urdf)
ament_export_dependencies(uos_common_urdf)
#ament_export_dependencies(sick_tim)
#ament_export_dependencies(velodyne_description)

ament_package()
