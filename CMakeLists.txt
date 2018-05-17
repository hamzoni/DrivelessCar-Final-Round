cmake_minimum_required(VERSION 2.8)

project( CarControl-0.3 )

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED)

set( OPENNI2_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/2-Linux/OpenNI-Linux-Arm-2.3/Include)
set( OPENNI2_REDIST  ${CMAKE_CURRENT_SOURCE_DIR}/2-Linux/OpenNI-Linux-Arm-2.3/Redist)
include_directories( ${OPENNI2_INCLUDE} )
link_directories ( ${OPENNI2_REDIST} )

#set(libname Haar)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
               STRINGS "Debug" "Release")
endif()
IF(MSVC)
  ADD_DEFINITIONS(-DNOMINMAX)
  # Another MSVC quirk. http://stackoverflow.com/questions/6832666/lnk2019-when-including-asio-headers-solution-generated-with-cmake
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc /MP") # For multi-threaded compilation on MSVC
ELSE()
  IF(APPLE)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++") # C++11 support on Apple clang
  ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")# C++11 support on GCC
  ENDIF()
ENDIF()

set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/libs)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CACHEFILE_DIR ${CMAKE_BINARY_DIR}/build)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/simd/src
    ${CMAKE_CURRENT_SOURCE_DIR}/stereo_vision
    ${CMAKE_CURRENT_SOURCE_DIR}/peripheral_driver/i2c
    ${CMAKE_CURRENT_SOURCE_DIR}/HAL
	${CMAKE_CURRENT_SOURCE_DIR}/openni2
	${CMAKE_CURRENT_SOURCE_DIR}/lane
	${CMAKE_CURRENT_SOURCE_DIR}/lane/laneclass
	${CMAKE_CURRENT_SOURCE_DIR}/lane/normal_lane
	${CMAKE_CURRENT_SOURCE_DIR}/lane/green_lane
	${CMAKE_CURRENT_SOURCE_DIR}/lane/secret_lane
	${CMAKE_CURRENT_SOURCE_DIR}/lane/fuzzy
	${CMAKE_CURRENT_SOURCE_DIR}/detector
	${CMAKE_CURRENT_SOURCE_DIR}/getconf
	${CMAKE_CURRENT_SOURCE_DIR}/haar2
		${CMAKE_CURRENT_SOURCE_DIR}/haar

)
#add_subdirectory(dlib build/dlib_build)
#add_subdirectory(simd/prj/cmake)
add_subdirectory(peripheral_driver)
add_subdirectory(stereo_vision)
add_subdirectory(HAL)
add_subdirectory(openni2)
add_subdirectory(getconf)
add_subdirectory(lane)
add_subdirectory(detector)
add_subdirectory(haar2)
add_subdirectory(haar)

#add_subdirectory(RTIMULib)


link_directories(
	${LIBRARY_OUTPUT_PATH}
	${CMAKE_CURRENT_SOURCE_DIR}/simd/prj/cmake/build
)

set(source
    t.h
    t.cpp
       )

add_library(tes ${source})

add_executable(lane auto.cpp)
target_link_libraries(lane  tes signhaar i2c-pwm kinect-cv2 HAL openni2 OpenNI2 lanedetec fuzzy objectdetec config  X11 ${OpenCV_LIBS} Simd -lpthread RTIMULib -pthread)

add_executable(debugLane debugLane.cpp)
target_link_libraries(debugLane  tes signhaar i2c-pwm kinect-cv2 HAL openni2 OpenNI2 lanedetec fuzzy objectdetec config  X11 ${OpenCV_LIBS} Simd -lpthread RTIMULib -pthread)

#add_executable(lane2 auto2.cpp)
#target_link_libraries(lane2  tes signhaar i2c-pwm kinect-cv2 HAL openni2 OpenNI2 lanedetec fuzzy objectdetec config  X11 ${OpenCV_LIBS} Simd -lpthread RTIMULib -pthread)


add_executable(testlane main/lane.cpp)
target_link_libraries(testlane lanedetec signhaar2 config ${OpenCV_LIBS} Simd)

add_executable(lane2 auto3.cpp)
target_link_libraries(lane2  tes signhaar2 i2c-pwm kinect-cv2 HAL openni2 OpenNI2 lanedetec fuzzy objectdetec config  X11 ${OpenCV_LIBS} Simd -lpthread RTIMULib -pthread)


#add_executable(lane2 auto2.cpp)
#target_link_libraries(lane2 signhaar i2c-pwm kinect-cv2 HAL openni2 OpenNI2 lanedetec fuzzy objectdetec config X11 ${OpenCV_LIBS} Simd -lpthread -pthread)


#add_executable(trafficsign main/traffic.cpp)
#target_link_libraries(trafficsign traffics config ${OpenCV_LIBS})

#add_executable(toolcolor main/testcannybinh.cpp)
#target_link_libraries(toolcolor ${OpenCV_LIBS})

#dd_executable(cam cam.cpp)
#arget_link_libraries(cam Haar Simd i2c-pwm kinect-cv2 HAL openni2 OpenNI2 config X11 ${OpenCV_LIBS})



