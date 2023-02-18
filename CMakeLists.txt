cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)
add_definitions(-std=c++11 -w -g)

set(PROJECT_NAME RandomGoalController)
set(PROJECT_DESCRIPTION "RandomGoalController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(Python COMPONENTS Interpreter Development)
# find_package(catkin REQUIRED COMPONENTS
#   moveit_core
#   moveit_msgs
#   moveit_ros_planning
#   moveit_ros_planning_interface
# )


add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/RandomGoalController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/RandomGoalController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/RandomGoalController.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/../../etc")

