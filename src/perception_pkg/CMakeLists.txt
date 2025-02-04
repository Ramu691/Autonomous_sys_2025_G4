cmake_minimum_required(VERSION 3.0.2)
project(perception_pkg)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        octomap_msgs
        octomap_ros
)

include_directories(${catkin_INCLUDE_DIRS})