cmake_minimum_required(VERSION 3.5)
project(metrics CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#find_package(yaml-cpp REQUIRED)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV")

#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/tkMetrics
                    ${OPENCV_INCLUDE_DIRS})

file(GLOB metrics-SRC "src/*.cpp")
set(metrics-LIBS yaml-cpp ${OpenCV_LIBS})
add_library(metrics SHARED ${metrics-SRC})
target_link_libraries(metrics ${metrics-LIBS})

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(tkMetrics_tracking demo/tracking.cpp)
target_link_libraries(tkMetrics_tracking metrics)

add_executable(tkMetrics_mAP demo/map.cpp)
target_link_libraries(tkMetrics_mAP metrics)