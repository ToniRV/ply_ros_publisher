cmake_minimum_required(VERSION 2.8.3)
project(ply_ros_publisher)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp)

cs_install()

cs_export()
