cmake_minimum_required(VERSION 3.0)

# Setup the project.
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(robot_control)


include_directories("common")
aux_source_directory(common COMMON_SRC)
include_directories("robot")
aux_source_directory(robot ROBOT_SRC)
include_directories("controller")
aux_source_directory(controller CTCONTROLLER_SRC)
include_directories("controller/FSM_States")
aux_source_directory(controller/FSM_States CTFSMCONTROLLER_SRC)
include_directories("/usr/local/include/eigen3")
include_directories("/usr/local/include/lcm")
include_directories("/opt/ros/noetic/include")
include_directories("./sim/gazebo/include")

file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

find_package(Boost)
find_package(catkin REQUIRED COMPONENTS
        controller_manager
        joint_state_controller
        gazebo_ros
        # gazebo
)
find_package(catkin REQUIRED COMPONENTS
        genmsg
        robot_state_publisher
        roscpp
        std_msgs
        tf
        geometry_msgs
        unitree_legged_msgs
)
catkin_package(
        CATKIN_DEPENDS
        unitree_legged_msgs
)
include_directories(
        #          include
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3")
link_directories(
        ${GAZEBO_LIBRARY_DIRS}
)

# ONNX
# 设置 ONNX Runtime 安装路径
set(ONNXRUNTIME_DIR "/usr/lib/onnxruntime-linux-x64")
# 包含头文件目录
include_directories(${ONNXRUNTIME_DIR}/include)
# 链接库文件目录
link_directories(${ONNXRUNTIME_DIR}/lib)

add_subdirectory(sim)

# Setup the target executable.
add_executable(robot_control ${SOURCES} ${ROBOT_SRC} ${COMMON_SRC} ${CTCONTROLLER_SRC} ${CTFSMCONTROLLER_SRC} ${RL_SRC})
target_link_libraries(robot_control  ${catkin_LIBRARIES})
target_link_libraries(robot_control  pthread lcm)
target_link_libraries(robot_control  ioros)
# ONNX
# 链接 ONNX Runtime 库
target_link_libraries(robot_control onnxruntime)
add_dependencies(robot_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#set_property(TARGET robot_control PROPERTY CXX_STANDARD 14)
