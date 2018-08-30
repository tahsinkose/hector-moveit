cmake_minimum_required(VERSION 2.8.3)
project(hector_moveit_gazebo_worlds)


set (CMAKE_CXX_STANDARD 11)
find_package(catkin REQUIRED)

catkin_package(
    DEPENDS
    CATKIN_DEPENDS
    INCLUDE_DIRS
    LIBRARIES
)

install(DIRECTORY
  Media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)