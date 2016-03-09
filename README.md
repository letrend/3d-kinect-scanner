#########################################################################
#         								#
#		KINECT FUSION PROJECT GPU PROGRAMMING SS15		#
#									#
#########################################################################
# authors:	Simon Trendel	-	simon.trendel(at)tum.de		#
#		Martin Herrmann	-	martin.herrmann(at)tum.de	#
#		Neeraj Sujan	- 	n.p.sujan(at)tum.de		#
#########################################################################		

#########################################################################
Dependencies:
	-	eigen3
	-	opengl
	-	glfw
	-	glew
	-	glm
	-	opencv
	-	cuda 6.5 and following
	-	libusb-1.0

#########################################################################
The source dir is referred to with ${SOURCE_DIR}.
${EIGEN_INCLUDE_DIR} 	= ${SOURCE_DIR}/third_party/include
${ICPCUDA_SRC} 		= ${SOURCE_DIR}/third_party/ICPCUDA/src

Install instructions:
	--libfreenect	
		cd ${SOURCE_DIR}/third_party/libfreenect
		mkdir build
		cd build
		mkdir release
		cd ${SOURCE_DIR}/third_party/libfreenect/build
		cmake ../ -DCMAKE_INSTALL_PREFIX=release
		make && make install
	--icpcuda (should install automatically via CMakeLists.txt, 
		   otherwise execute the following command in one line)
		nvcc --compiler-options '-fPIC' --compiler-options '-O3' -o ${SOURCE_DIR}/third_party/lib/libicpodometry.so
		--shared ${ICPCUDA_SRC}/ICPOdometry.cpp ${ICPCUDA_SRC}/Cuda/*.cu ${ICPCUDA_SRC}/Cuda/containers/*.cpp
		-I${EIGEN_INCLUDE_DIR}/ -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lcudadevrt 
		--use_fast_math -gencode=arch=compute_20,code=sm_20 -gencode=arch=compute_30,code=sm_30 
		-gencode=arch=compute_35,code=sm_35 -gencode=arch=compute_50,code=sm_50
	--tsdf_cuda (the actual kinect fusion project)
		cd ${SOURCE_DIR}
		cmake .
		make
	
Usage:
	--run on rgbd_dataset (example):
		./tsdf_cuda -i ../rgbd_dataset_freiburg3_teddy/ 

	--run with kinect (example):
		./tsdf_cuda -i ../mesh_output/ -k
	--for more commandline parameters run without arguments
		./tsdf_cuda

			
		



	
