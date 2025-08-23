include_directories("/usr/local/include/eigen3")
include_directories(${PROJECT_SOURCE_DIR})
include_directories("include")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

SET(C_SOURCES "./src/interface/IOROS.cpp")
add_library(ioros SHARED ${C_SOURCES})
target_link_libraries (ioros ${catkin_LIBRARIES})