cmake_minimum_required(VERSION 2.8.3)
project(hector_moveit_exploration)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    moveit_core 
    moveit_ros_planning_interface 
    std_msgs 
    hector_uav_msgs
    actionlib
)


catkin_package(CATKIN_DEPENDS roscpp moveit_core moveit_ros_planning_interface std_msgs hector_uav_msgs)


include_directories(${catkin_INCLUDE_DIRS})

add_executable(hector_explorer src/explorer.cpp)
target_link_libraries(hector_explorer ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(hector_explorer hector_uav_msgs_generate_messages)
