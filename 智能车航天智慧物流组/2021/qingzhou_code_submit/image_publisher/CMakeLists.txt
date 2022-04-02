cmake_minimum_required(VERSION 3.10)

project(image_publisher)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	image_transport
	sensor_msgs
)

catkin_package()

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

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
)

