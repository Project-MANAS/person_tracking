cmake_minimum_required(VERSION 3.5)
project(person_tracking)

set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        pcl_ros
        pcl_conversions
)

catkin_package(CATKIN_DEPENDS
        roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        visualization_msgs
        pcl_ros
        pcl_conversions
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/PersonTracker.cpp
)

add_executable(person_tracker src/person_tracker_node.cpp)
target_link_libraries(person_tracker ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(person_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
