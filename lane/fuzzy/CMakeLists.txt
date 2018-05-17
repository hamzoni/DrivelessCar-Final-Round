cmake_minimum_required(VERSION 2.8)
project(config)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(source_file
	fuzzylogic.h
	fuzzylogic.cpp)

add_library(fuzzy ${source_file})
