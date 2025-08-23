cmake_minimum_required(VERSION 3.0.2)
project(robot_cmd)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        unitree_legged_msgs
)

catkin_package(
        INCLUDE_DIRS
        CATKIN_DEPENDS
        roscpp
        unitree_legged_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/teleop_node.cpp)

add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
)