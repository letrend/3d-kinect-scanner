cmake_minimum_required(VERSION 2.6.0)

project(kinect_scanner)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

#==============================================================================
# Dependencies
#==============================================================================s
# catkin
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs message_generation rviz)
#generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime message_generation)

#qt4
find_package(Qt4)
find_package(Protobuf REQUIRED)
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

# Kinect
find_package(freenect2 REQUIRED)
include_directories(${freenect2_INCLUDE_DIR})

# CUDA
SET(CUDA_TOOLKIT_ROOT_DIR /opt/cuda)
FIND_PACKAGE(CUDA REQUIRED)
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -gencode=arch=compute_50,code=sm_50 --use_fast_math)# -Xcompiler -DDEBUG
#-Xcompiler -g
# preprocessor define: source directory
ADD_DEFINITIONS(-DTSDF_CUDA_SOURCE_DIR=${PROJECT_SOURCE_DIR})

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

# OpenCV
FIND_PACKAGE(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_calib3d REQUIRED)

file(GLOB srcs ${CMAKE_CURRENT_SOURCE_DIR}/../../ICPCUDA/src/ICPOdometry.cpp)
file(GLOB cuda ${CMAKE_CURRENT_SOURCE_DIR}/../../ICPCUDA/src/Cuda/*.cu)
file(GLOB containers ${CMAKE_CURRENT_SOURCE_DIR}/../../ICPCUDA/src/Cuda/containers/*.cpp)

# Eigen
find_package(PkgConfig)
pkg_check_modules(EIGEN3 REQUIRED eigen3)
message(STATUS ${EIGEN3_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories( include/
	${CMAKE_CURRENT_SOURCE_DIR}/../../ICPCUDA/src
	${CMAKE_CURRENT_SOURCE_DIR}/../../kinect_utilities/include
        )

#==============================================================================
# Application
#==============================================================================s

qt4_wrap_cpp(MOC_FILES include/kinectScannerPlugin.hpp)
CUDA_ADD_LIBRARY(kinect_scanner_plugin src/kinectScannerPlugin.cpp ${srcs} ${cuda} ${containers} ${MOC_FILES})
target_link_libraries(kinect_scanner_plugin ${QT_LIBRARIES} ${catkin_LIBRARIES} ${freenect2_LIBRARY} ${OpenCV_LIBS} )

