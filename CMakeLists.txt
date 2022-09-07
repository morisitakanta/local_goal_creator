cmake_minimum_required(VERSION 3.0.2)
project(local_goal_creator)

find_package(catkin REQUIRED COMPONENTS
roscpp
amsl_navigation_msgs
)

catkin_package(
INCLUDE_DIRS include
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/local_goal_creator_node.cpp src/local_goal_creator.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})