# - Find the libfreenect includes and library
# This module defines
#  FREENECT_INCLUDE_DIRS, path to libfreenect/libfreenect.h, etc.
#  FREENECT_LIBRARIES, the libraries required to use FREENECT.
#  FREENECT_FOUND, If false, do not try to use FREENECT.
# also defined, but not for general use are
#  FREENECT_freenect_LIBRARY, where to find the FREENECT library.
# Adapted from: https://github.com/jtbates/lua---kinect/blob/master/FindFreenect.cmake

FIND_LIBRARY(FREENECT_LIBRARIES freenect freenect_sync
	${CMAKE_SOURCE_DIR}/third_party/libfreenect/build/lib
	NO_DEFAULT_PATH
)

SET(FREENECT_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/third_party/libfreenect/include ${CMAKE_SOURCE_DIR}/third_party/libfreenect/wrappers/cpp)

MARK_AS_ADVANCED(
  FREENECT_INCLUDE_DIRS
  FREENECT_LIBRARIES)

SET( FREENECT_FOUND "NO" )
IF(FREENECT_INCLUDE_DIRS)
  IF(FREENECT_LIBRARIES)
    SET( FREENECT_FOUND "YES" )
    SET( FREENECT_LIBRARIES
      ${FREENECT_LIBRARIES})
  ENDIF(FREENECT_LIBRARIES)
ENDIF(FREENECT_INCLUDE_DIRS)

IF(FREENECT_FOUND)
    MESSAGE(STATUS "Found freenect library")
    MESSAGE(STATUS "freenect include dir: ${FREENECT_INCLUDE_DIRS}" )
    MESSAGE(STATUS "freenect library: ${FREENECT_LIBRARIES}" )
ELSE(FREENECT_FOUND)
  IF(FREENECT_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find libfreenect 
-- please give some paths to CMake or make sure libfreenect is installed in your system")
  ELSE(FREENECT_FIND_REQUIRED)
    MESSAGE(STATUS "Could not find libfreenect 
-- please give some paths to CMake or make sure libfreenect is installed in your system")
  ENDIF(FREENECT_FIND_REQUIRED)
ENDIF(FREENECT_FOUND)
