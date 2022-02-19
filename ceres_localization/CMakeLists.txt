cmake_minimum_required(VERSION 2.8.3)
project(ceres_localization)

find_package(catkin REQUIRED)
catkin_package()

# install(PROGRAMS
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)