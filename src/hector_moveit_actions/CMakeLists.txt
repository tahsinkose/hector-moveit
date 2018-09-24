cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(hector_moveit_actions)

find_package(catkin REQUIRED COMPONENTS roscpp actionlib actionlib_msgs geometry_msgs)
find_package(Boost REQUIRED COMPONENTS system)
add_action_files(
  DIRECTORY action
  FILES ExecuteDroneTrajectory.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs geometry_msgs
)
catkin_package(
    CATKIN_DEPENDS roscpp actionlib_msgs
)


include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(trajectory_executer src/trajectory_executer.cpp)
target_link_libraries(trajectory_executer ${catkin_LIBRARIES})

add_dependencies(
  trajectory_executer
  ${hector_moveit_actions_EXPORTED_TARGETS}
)