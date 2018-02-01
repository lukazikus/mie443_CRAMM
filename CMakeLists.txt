cmake_minimum_required(VERSION 2.8)
project(mie443_contest1)

#set(OpenCV_DIR "/usr/share/OpenCV")

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	genmsg
	image_transport
	sensor_msgs
	kobuki_msgs)

generate_messages(DEPENDENCIES sensor_msgs kobuki_msgs)

catkin_package()

include_directories(include ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add the A* library
add_library(a_star src/a_star.cpp)
target_link_libraries(a_star ${catkin_LIBRARIES} ${OpenCV_LIB})

# add the publisher example
add_executable(contest1 src/contest1.cpp)
target_link_libraries(contest1 a_star ${catkin_LIBRARIES} ${OpenCV_LIB})
