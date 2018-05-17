cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED)

set(SOURCES
	haar.h
	haar.cpp
)
add_library(signhaar ${SOURCES})
