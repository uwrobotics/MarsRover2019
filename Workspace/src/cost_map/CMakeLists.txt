cmake_minimum_required(VERSION 3.12)
project(cost_map_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# Where to find CMake modules and OpenCV
set(OpenCV_DIR opencv_2/mingw-build/install)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(OpenCV REQUIRED HINTS opencv_2/mingw-build/install)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(cost_map_node cost_map_node.cpp
        cost_map_locatlization.cpp
        cost_map_localization.h)

# add libs you need
set(OpenCV_LIBS opencv_core opencv_imgproc)

# linking
target_link_libraries(cost_map_node ${OpenCV_LIBS})

