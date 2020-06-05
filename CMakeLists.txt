cmake_minimum_required(VERSION 3.5)
project(metrics CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 ")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    /usr/local/MATLAB/R2018b/extern/include/)

file(GLOB metrics-SRC "src/*.cpp")
#set(metrics-LIBS)
add_library(metrics SHARED ${metrics-SRC})
target_link_libraries(metrics ${metrics-LIBS} /usr/local/MATLAB/R2018b/bin/glnxa64/libmex.so )

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(main demo/main.cpp)
target_link_libraries(main metrics)