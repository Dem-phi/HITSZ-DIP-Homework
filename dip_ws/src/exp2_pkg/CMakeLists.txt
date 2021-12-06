cmake_minimum_required(VERSION 3.0.2)
project(exp2_pkg)
set(OpenCV_DIR /usr/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        OpenCV
        )
catkin_package()
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
add_executable(demo1
        src/demo.cpp
        )
target_link_libraries(demo1
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})