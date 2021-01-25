cmake_minimum_required(VERSION 3.5)
project(tkMetrics CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

#-------------------------------------------------------------------------------
# External Libraries
#-------------------------------------------------------------------------------

#find_package(yaml-cpp REQUIRED)

find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV")

#-------------------------------------------------------------------------------
# Build libraries
#-------------------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/tkMetrics
                    ${OPENCV_INCLUDE_DIRS})

file(GLOB tkMetrics-SRC "src/*.cpp")
set(tkMetrics-LIBS yaml-cpp ${OpenCV_LIBS})
add_library(tkMetrics SHARED ${tkMetrics-SRC})
target_link_libraries(tkMetrics ${tkMetrics-LIBS})

#-------------------------------------------------------------------------------
# Build executables
#-------------------------------------------------------------------------------
add_executable(tkMetrics_tracking demo/tracking.cpp)
target_link_libraries(tkMetrics_tracking tkMetrics)

add_executable(tkMetrics_mAP demo/map.cpp)
target_link_libraries(tkMetrics_mAP tkMetrics)