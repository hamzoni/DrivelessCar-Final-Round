cmake_minimum_required(VERSION 2.8)

project (Traffic)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(source_files
	svmdetector.h
	svmdetector.cpp)

add_library(traffic ${source_files})

