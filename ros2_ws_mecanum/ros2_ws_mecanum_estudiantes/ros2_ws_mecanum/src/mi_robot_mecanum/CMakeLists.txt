cmake_minimum_required(VERSION 3.8)
project(mi_robot_mecanum)

find_package(ament_cmake REQUIRED)

# Carpetas que se copian a install/share/mi_robot_mecanum
install(DIRECTORY urdf worlds launch rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
