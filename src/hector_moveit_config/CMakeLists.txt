cmake_minimum_required(VERSION 2.8.3)
project(hector_moveit_config)

set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
            roscpp std_msgs hector_uav_msgs)


catkin_package(CATKIN_DEPENDS roscpp std_msgs hector_uav_msgs)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop ${catkin_LIBRARIES})
add_dependencies(keyboard_teleop hector_uav_msgs_generate_messages)

