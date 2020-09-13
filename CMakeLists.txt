cmake_minimum_required(VERSION 2.8.3)
project(mursim_actuator)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    mur_common
    )

catkin_package()

add_definitions(-std=c++14)
include_directories(src ${catkin_INCLUDE_DIRS})

add_executable(actuator_node src/actuator_node.cpp)

target_link_libraries(actuator_node ${catkin_LIBRARIES})
