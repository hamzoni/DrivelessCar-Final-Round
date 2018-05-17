cmake_minimum_required(VERSION 2.8)

project (GreenLaneDetection)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(source_files
        greenimageprocess.h
        greenimageprocess.cpp
        greenlaneprocess.h
        greenlaneprocess.cpp
)

add_library(green_lane ${source_files})

