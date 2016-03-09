CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(3D-kinect-scanner C CXX)

# RelWithDebInfo, Release or Debug
SET(CMAKE_BUILD_TYPE Debug FORCE)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
#==============================================================================
# Dependencies
#==============================================================================

# Eigen
SET(EIGEN_ROOT ${PROJECT_SOURCE_DIR}/third_party/include/)
FIND_PACKAGE(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIR})

# CUDA
SET(CUDA_TOOLKIT_ROOT_DIR /usr/local/cuda-7.5)
FIND_PACKAGE(CUDA REQUIRED)
SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -Xcompiler -g -gencode=arch=compute_50,code=sm_50 --use_fast_math)# -Xcompiler -DDEBUG -Xcompiler -g

# OpenCV
FIND_PACKAGE(OpenCV COMPONENTS opencv_core opencv_highgui opencv_imgproc REQUIRED)

# Kinect
find_package(Freenect REQUIRED )
INCLUDE_DIRECTORIES(${FREENECT_INCLUDE_DIRS})
message(STATUS  ${FREENECT_INCLUDE_DIRS})

# libusb
FIND_PACKAGE(Libusb REQUIRED)
INCLUDE_DIRECTORIES(${LIBUSB_INCLUDE_DIRS})

# ncurses
FIND_PACKAGE(Curses REQUIRED)

#==============================================================================
# Application
#==============================================================================

# preprocessor define: source directory
ADD_DEFINITIONS(-DTSDF_CUDA_SOURCE_DIR=${PROJECT_SOURCE_DIR})

# application
INCLUDE_DIRECTORIES(include)

set(ICPCUDALIB_DIR "${CMAKE_SOURCE_DIR}/third_party/lib")
find_library(ICPCUDAODOMETRY_LIB NAMES icpodometry PATHS ${ICPCUDALIB_DIR})

IF(ICPCUDAODOMETRY_LIB)
    MESSAGE(STATUS "Found icpcuda library ${ICPCUDAODOMETRY_LIB}")
ELSE(ICPCUDAODOMETRY_LIB)
    MESSAGE(STATUS "Can not find icpcuda library, I will try to build it for you")
    set(ICPCUDA_SRC "${CMAKE_SOURCE_DIR}/third_party/ICPCUDA/src/")
    EXEC_PROGRAM(nvcc ARGS --compiler-options '-fPIC' --compiler-options '-O3' -o ${CMAKE_SOURCE_DIR}/third_party/lib/libicpodometry.so 
            --shared ${ICPCUDA_SRC}/ICPOdometry.cpp ${ICPCUDA_SRC}/Cuda/*.cu ${ICPCUDA_SRC}/Cuda/containers/*.cpp 
            -I${EIGEN_INCLUDE_DIR}/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lcudadevrt 
            -lcudart --use_fast_math -gencode=arch=compute_20,code=sm_20 -gencode=arch=compute_30,code=sm_30 
            -gencode=arch=compute_35,code=sm_35 -gencode=arch=compute_50,code=sm_50 2>&1 OUTPUT_VARIABLE SUCCESS)
    
    IF(SUCCESS STREQUAL "")
        MESSAGE(STATUS "I was able to build it")
        set(ICPCUDAODOMETRY_LIB "${CMAKE_SOURCE_DIR}/third_party/lib/libicpodometry.so")
    ELSE(ICPCUDAODOMETRY_LIB)
        MESSAGE(STATUS "Something went wrong, maybe you are missing some dependencies")
        MESSAGE(FATAL_ERROR ${SUCCESS})
    ENDIF()
ENDIF(ICPCUDAODOMETRY_LIB)

CUDA_ADD_EXECUTABLE(3D-kinect-scanner src/main.cu src/marching_cubes.cpp src/VolumeIntegration.cu)
TARGET_LINK_LIBRARIES(3D-kinect-scanner ${OpenCV_LIBS} ${ICPCUDAODOMETRY_LIB} ${FREENECT_LIBRARIES} ${CURSES_LIBRARIES} -lboost_system -lpthread)

