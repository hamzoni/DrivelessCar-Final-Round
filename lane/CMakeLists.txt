cmake_minimum_required(VERSION 2.8)

project (LaneDetection)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_subdirectory(laneclass)
add_subdirectory(normal_lane)
add_subdirectory(green_lane)
add_subdirectory(fuzzy)
add_subdirectory(secret_lane)

set(source_files
	debug.h
	debug.cpp
	green.h
	green.cpp
	frameprocess.h
	frameprocess.cpp
	status.cpp
	status.h
	secret.h
	secret.cpp
)

add_library(lanedetec ${source_files})
target_link_libraries(lanedetec secret green_lane normal_lane  laneclass fuzzy)
