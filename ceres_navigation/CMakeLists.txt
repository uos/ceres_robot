cmake_minimum_required(VERSION 2.8.3)
project(ceres_navigation)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS
  scripts/mbf_move_base_action.py
  scripts/mbf_state_machine.py
  scripts/plan_exec_sm.py
  scripts/planner_sm.py
  scripts/smach_polyfill.py
  scripts/wait_for_goal.py
  scripts/navigation_sm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

