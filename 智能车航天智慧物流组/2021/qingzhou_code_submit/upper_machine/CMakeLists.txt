cmake_minimum_required(VERSION 3.10)

project(upper_machine)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	move_base_msgs
	actionlib
	sensor_msgs
	cv_bridge
	image_transport
	tf2_ros
	tf2_geometry_msgs
)

catkin_package()

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

file(GLOB_RECURSE cxx_src "src/*.h" "src/*.hpp" "src/*.cpp")

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} ${cxx_src})
target_link_libraries(${PROJECT_NAME}
	PRIVATE
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
    Qt5::Widgets
    Threads::Threads
)
