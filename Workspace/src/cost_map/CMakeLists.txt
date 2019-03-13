cmake_minimum_required(VERSION 2.8.3)
project(cost_map)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        #OpenCV
        #image_geometry
        cv_bridge
        std_msgs
        sensor_msgs
        obstacle_detection
        message_generation
        )

find_package(OpenCV REQUIRED)

add_message_files(
        FILES
        Costmap.msg
)
generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
)
catkin_package(
        CATKIN_DEPENDS roscpp std_msgs message_runtime
)

#find_package(OpenCV REQUIRED HINTS opencv_2/mingw-build/install)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node cost_map_node.cpp
        cost_map_locatlization.cpp
        )

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} cost_map_generate_messages)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES} ${OpenCV_LIBS}
        )

# add libs you need
#set(OpenCV_LIBS opencv_core opencv_imgproc)

# linking
#target_link_libraries(cost_map_node ${OpenCV_LIBS})
