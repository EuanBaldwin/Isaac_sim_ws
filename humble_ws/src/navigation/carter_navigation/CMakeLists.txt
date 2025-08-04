cmake_minimum_required(VERSION 3.5)
project(carter_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  params
  maps
  rviz2
#  behavior_trees
  DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS
    scripts/logger.py
    scripts/battery_sim.py
    scripts/tag_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

