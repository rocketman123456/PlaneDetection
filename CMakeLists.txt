cmake_minimum_required (VERSION 3.1)

project (RGBDPlaneDetection)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")

# put FindGLFW.cmake in the source directory of CMakeLists.txt
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
#include_directories(PEAC/include)

file(GLOB MRF_src "MRF2.2/src/*.cpp")
add_library(MRF ${MRF_src})
target_include_directories(MRF PUBLIC
    MRF2.2/include
)

#file(GLOB RSPD_src "RSPD/src/*.cpp")
#add_library(RSPD ${RSPD_src})
#target_include_directories(RSPD PUBLIC
#    RSPD/include
#)

add_executable(peac_test PEAC/src/peac_test.cpp PEAC/src/plane_detection.cpp)
target_link_libraries(peac_test ${OpenCV_LIBS} realsense2 MRF)
target_include_directories(peac_test PRIVATE PEAC/include)

add_executable(local_map_test local_map/src/local_map_test.cpp local_map/src/local_map.cpp)
target_link_libraries(local_map_test ${OpenCV_LIBS} realsense2)
target_include_directories(local_map_test PRIVATE local_map/include)
