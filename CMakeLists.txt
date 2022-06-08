CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
PROJECT(key_points_planner)

SET(CMAKE_CXX_STANDARD 14)

FIND_PACKAGE(
        catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_core
        tf2_ros
)

CATKIN_PACKAGE()

INCLUDE_DIRECTORIES(
        include
        ${catkin_INCLUDE_DIRS}
)

ADD_EXECUTABLE(key_points_planner_node src/key_points_planner_node.cpp)
TARGET_LINK_LIBRARIES(key_points_planner_node ${catkin_LIBRARIES})

ADD_LIBRARY(key_points_planner src/key_points_planner_plugin.cpp)
TARGET_LINK_LIBRARIES(key_points_planner ${catkin_LIBRARIES})
