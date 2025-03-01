cmake_minimum_required(VERSION 3.0.2)
project(trajectory_pkg)
set (CMAKE_CXX_STANDARD 11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(octomap REQUIRED) 
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap_msgs
  octomap_ros
  tf
  tf_conversions
  visualization_msgs
  message_generation
  custom_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs octomap_msgs octomap_ros cv_bridge eigen_conversions fla_msgs geometry_msgs image_transport mav_msgs message_generation nav_msgs roscpp rospy sensor_msgs std_msgs tf tf_conversion message_runtime visualization_msgs message_runtime

#  DEPENDS system_lib
)
###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_libraries(${OCTOMAP_LIBRARIES})

add_executable(traj_gen src/traj_gen.cpp)
add_dependencies(traj_gen ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_gen ${catkin_LIBRARIES})


## Specify libraries to link a library or executable target against

target_link_libraries(traj_gen
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)


