cmake_minimum_required(VERSION 2.8.3)
project(hector_moveit_exploration)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    moveit_core 
    moveit_ros_planning_interface 
    std_msgs 
    hector_uav_msgs
    hector_moveit_actions
    actionlib
)

find_package(octomap REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface std_msgs hector_uav_msgs hector_moveit_actions)


include_directories(include ${OCTOMAP_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(hector_explorer src/Explorer.cpp src/explorer_node.cpp)
target_link_libraries(hector_explorer ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(hector_explorer hector_uav_msgs_generate_messages hector_moveit_actions_generate_messages)
