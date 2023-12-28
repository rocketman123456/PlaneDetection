cmake_minimum_required (VERSION 3.1)

project (RGBDPlaneDetection)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-fpermissive")
set(CMAKE_CXX_COMPILER /usr/bin/g++)
set(CMAKE_BUILD_TYPE "Release")

# put FindGLFW.cmake in the source directory of CMakeLists.txt
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS})
include_directories(include/peac)
include_directories(include/MRF2.2)
include_directories(include)

file(GLOB MRF_src "peac/src/MRF2.2/*.cpp")

add_library(MRF ${MRF_src})
target_include_directories(MRF PUBLIC
    peac/include/MRF2.2
)

add_executable(peac_test peac/src/peac_test.cpp peac/src/plane_detection.cpp)
target_link_libraries(peac_test ${OpenCV_LIBS} realsense2 MRF)
target_include_directories(peac_test PUBLIC
    peac/include/peac
    peac/include/MRF2.2
    peac/include
)

add_executable(local_map_test local_map/src/local_map_test.cpp)
target_link_libraries(local_map_test ${OpenCV_LIBS} realsense2)
target_include_directories(local_map_test PUBLIC
    local_map/include
)
