cmake_minimum_required(VERSION 3.0.2)
project(exp1_pkg)
set(OpenCV_DIR /usr/share/OpenCV)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        OpenCV
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(demo src/demo.cpp)
target_link_libraries(demo ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(demo2 src/demo2.cpp)
target_link_libraries(demo2 ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_executable(demo3 src/demo3.cpp)
target_link_libraries(demo3 ${catkin_LIBRARIES} ${OpenCV_LIBS})