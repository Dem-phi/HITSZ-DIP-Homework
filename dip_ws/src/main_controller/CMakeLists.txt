cmake_minimum_required(VERSION 3.0.2)
project(main_controller)
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

add_executable(final
        src/final.cpp
        )
target_link_libraries(final
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

add_executable(rviz_visualize
        src/rviz_visualize.cpp
        )
target_link_libraries(rviz_visualize
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

add_executable(test_planning
        src/test.cpp
        )
target_link_libraries(test_planning
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})
