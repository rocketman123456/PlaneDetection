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

file(GLOB MRF_src "src/MRF2.2/*.cpp")
file(GLOB sources "src/main.cpp" "src/plane_detection.cpp")

add_library(MRF ${MRF_src})

add_executable(RGBDPlaneDetection ${sources})
target_link_libraries(RGBDPlaneDetection ${OpenCV_LIBS} MRF)

add_executable(realsense_test src/realsense_test.cpp src/plane_detection.cpp)
target_link_libraries(realsense_test ${OpenCV_LIBS} realsense2 MRF)
