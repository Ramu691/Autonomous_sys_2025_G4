cmake_minimum_required(VERSION 3.0.2)
project(system)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mav_msgs
  geometry_msgs
  message_generation
  fla_msgs
  message_runtime
  custom_msgs
  mav_state_machine_msgs  # This will find the mav_state_machine_msgs package
)

## Add message files from fla_msgs
add_message_files(
  DIRECTORY fla_msgs/msg
  FILES
  Box.msg
  ControlMessage.msg
  Detection.msg
  FlightCommand.msg
  FlightEvent.msg
  FlightStateTransition.msg
  ImageDetections.msg
  ImageSegmentation.msg
  JoyDef.msg
  Keypoint.msg
  Latency.msg
  NodeList.msg
  NodeStatus.msg
  ProcessStatus.msg
  TelemString.msg
  WaypointList.msg
)

get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)

## Add service files from mav_state_machine_msgs
add_service_files(
  DIRECTORY ${PARENT_DIR}/mav_comm/mav_state_machine_msgs/srv
  FILES
  RunTaskService.srv
)

add_message_files(
  DIRECTORY ${PARENT_DIR}/mav_comm/mav_state_machine_msgs/msg
  FILES
  StartStopTask.msg
)

## Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  mav_state_machine_msgs  # Ensure this is included
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs geometry_msgs fla_msgs custom_msgs mav_state_machine_msgs
)

## Include directories for custom messages and mav_state_machine_msgs
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${custom_msgs_INCLUDE_DIRS}
  ${mav_state_machine_msgs_INCLUDE_DIRS}
  
)

## Declare the state machine node
add_executable(state_machine_node src/state_machine_node.cpp)


## Add dependencies for message generation
add_dependencies(state_machine_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Link libraries to the node
target_link_libraries(state_machine_node
  ${catkin_LIBRARIES} ${custom_msgs_LIBRARIES}
)




