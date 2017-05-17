CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(3D-kinect-scanner C CXX)

add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 ")

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#==============================================================================
# Dependencies
#==============================================================================

# Eigen
find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
message(STATUS ${EIGEN3_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

# CUDA
SET(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-8.0)
FIND_PACKAGE(CUDA REQUIRED)
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -Xcompiler -g -gencode=arch=compute_61,code=sm_61  --use_fast_math -Xcompiler
        -O3)# -Xcompiler -DDEBUG -Xcompiler -g

# OpenCV
FIND_PACKAGE(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

# Kinect
find_package(freenect2 REQUIRED)
include_directories(${freenect2_INCLUDE_DIR})

# ncurses
FIND_PACKAGE(Curses REQUIRED)

#==============================================================================
# Application
#==============================================================================

# preprocessor define: source directory
ADD_DEFINITIONS(-DTSDF_CUDA_SOURCE_DIR=${PROJECT_SOURCE_DIR})

# application
INCLUDE_DIRECTORIES(include kinect_utilities/include ICPCUDA/src)

file(GLOB srcs ICPCUDA/src/ICPOdometry.cpp)
file(GLOB cuda ICPCUDA/src/Cuda/*.cu)
file(GLOB containers ICPCUDA/src/Cuda/containers/*.cpp)

CUDA_ADD_EXECUTABLE(3D-kinect-scanner src/main.cu src/marching_cubes.cpp src/VolumeIntegration.cu src/calibration.cpp
        ${srcs} ${cuda} ${containers})
TARGET_LINK_LIBRARIES(3D-kinect-scanner ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${CURSES_LIBRARIES} -lboost_system -lpthread)

