cmake_minimum_required(VERSION 3.0.2)
project(exp3_pkg)
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
add_executable(demo33
        src/demo.cpp
        )
target_link_libraries(demo33
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

add_executable(homework_8
        src/homework_8.cpp
        )
target_link_libraries(homework_8
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})