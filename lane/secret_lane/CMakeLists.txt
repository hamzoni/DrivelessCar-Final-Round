cmake_minimum_required(VERSION 2.8)

project (SecretLaneDetection)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(source_files
        secretlaneprocess.h
        secretlaneprocess.cpp
)

add_library(secret ${source_files})

