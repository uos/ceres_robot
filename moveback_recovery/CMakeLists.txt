cmake_minimum_required(VERSION 2.8.3)
project(moveback_recovery)

find_package(catkin REQUIRED
    COMPONENTS
    cmake_modules
    roscpp
    tf
    costmap_2d
    mbf_costmap_core
    pluginlib
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        pluginlib
)

add_library(${PROJECT_NAME} src/moveback_recovery.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES moveback_recovery_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
