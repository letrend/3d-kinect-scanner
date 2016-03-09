# 3D-KINECT-SCANNER
cuda implementation of a 3d scanner for the kinect v1 

## DEPENDENCIES ##
eigen
cuda 7.5
opencv
freenect 
libusb
ncurses

## BUILD ##
```
#!bash
cd path/to/3d-kinect-scanner
mkdir build data
cd build
cmake ..
make -j4
```

## RUN ##
```
#!bash
./3D-kinect-scanner
```

## USAGE ##
### initialize ###
use initialize to set the cube size and the voxel size. this affects the scanable objectsize.
it is however limited by the available memory on your gpu. for 2GB 500x500x500 seems to be the limit.
the voxelsize affects the resolution of the resulting mesh.
### scan ###
scan what you like
### save mesh ###
choose filename or default. you will find the result in path/to/3d-kinect-scanner/data
