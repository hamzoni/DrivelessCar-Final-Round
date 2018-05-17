cmake_minimum_required(VERSION 2.8)

project (NormalLaneDetection)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(source_files
	laneimageprocess.h
	laneimageprocess.cpp
	laneprocess.h
	laneprocess.cpp
	lineprocess.h
	lineprocess.cpp
	pointsprocess.h
	pointsprocess.cpp
)

add_library(normal_lane ${source_files})

