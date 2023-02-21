cmake_minimum_required(VERSION 3.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 14)
set(ENV{CUDACXX} "/usr/local/cuda-11.6/bin/nvcc")
add_definitions(-std=c++14 -w -g)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(PROJECT_NAME Controllers)
set(PROJECT_DESCRIPTION "Controllers")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)
find_package(roscpp REQUIRED)
find_package(check_selfcollision REQUIRED HINTS "/home/laptop/catkin_ws/devel")

# add_subdirectory(random_goal_controller)
# add_subdirectory(residual_learning_controller)
add_subdirectory(bolotnikova_controller)