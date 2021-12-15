cmake_minimum_required(VERSION 2.8.3)
project(mycobot_320_moveit)

find_package(catkin REQUIRED
actionlib
actionlib_msgs
std_msgs
message_generation
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Move.action
  MultiMove.action
  MoveHome.action
)

add_message_files(
  FILES
  robot_goals.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)
catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
