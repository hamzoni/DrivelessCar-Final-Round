cmake_minimum_required(VERSION 2.8)

project(DetectecObject)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( OpenCV REQUIRED )
set(source_files
	detectorobj.h
	detectorobj.cpp)

add_library(objectdetec ${source_files})
