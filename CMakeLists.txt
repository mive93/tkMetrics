cmake_minimum_required(VERSION 3.5)
project(metrics CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/tkMetrics)

file(GLOB metrics-SRC "src/*.cpp")
#set(metrics-LIBS)
add_library(metrics SHARED ${metrics-SRC})
#target_link_libraries(metrics ${metrics-LIBS})

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(tkMetrics_tracking demo/main.cpp)
target_link_libraries(tkMetrics_tracking metrics)