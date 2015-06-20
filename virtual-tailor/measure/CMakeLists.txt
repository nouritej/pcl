cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(main)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
set(HEADER_FILES "src/segmentation.h"
"src/boundy.h"
"src/rotate.h"
"src/filter.h"
"src/cluster.h"
"src/section.h"
"src/icp.h"
"src/convexcave.h"
"src/measure.h")

file(GLOB planar_segmentation_SRC
    "src/*.h"
    "src/*.cpp"
)
add_library(mylib src/segmentation.cpp src/rotate.cpp src/boundy.cpp src/filter.cpp src/cluster.cpp src/section.cpp src/icp.cpp src/convexcave.cpp src/measure.cpp ${HEADER_FILES})
add_executable (main main.cpp ${HEADER_FILES})
target_link_libraries (main ${PCL_LIBRARIES} mylib)
