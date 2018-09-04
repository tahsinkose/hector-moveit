cmake_minimum_required(VERSION 2.8.3)
project(hector_moveit_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    pluginlib)

find_package(gazebo REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp pluginlib
)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")


add_library(fruit_generator SHARED src/fruit_generator.cc)
target_link_libraries(fruit_generator ${GAZEBO_LIBRARIES})

