cmake_minimum_required(VERSION 2.8)

project (LaneClass)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(source_files
	lane.h
	lane.cpp
	layer.h
	layer.cpp
	line.h
	line.cpp
	road.h
	road.cpp
	roadmultilanes.cpp
	roadmultilanes.h)

add_library(laneclass ${source_files})

