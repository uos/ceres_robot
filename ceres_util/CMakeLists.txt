cmake_minimum_required(VERSION 3.8)
project(ceres_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
ament_environment_hooks(env_hooks/iamceres.sh.in)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
