cmake_minimum_required(VERSION 3.0.2)
project(exp4_pkg)
set(OpenCV_DIR /usr/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        OpenCV
        )
catkin_package()
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
add_executable(exp4
        src/exp4.cpp
        )
target_link_libraries(exp4
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

add_executable(Color
        src/Color.cpp
        )
target_link_libraries(Color
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})