cmake_minimum_required(VERSION 2.8.3)
project(walker_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    geometry_msgs 
    move_base_msgs
    sensor_msgs
)


### Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(FILES change_string.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

# Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walker src/main.cpp src/walker.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})