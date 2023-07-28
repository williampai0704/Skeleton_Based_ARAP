# Skeleton_Based_ARAP

This project implemented a skeleton based ARAP (As Rigid As Possible) deformation algorithm on animal 3d mesh.

## Dependencies
This project uses [Eigen3](https://gitlab.com/libeigen/eigen) library in version 3.4.0. 
Mac users can also install by 
```
brew install eigen
```
remember to change the Eigen3_DIR in CMakeList.txt to correspond path.

Also, [libigl](https://github.com/libigl/libigl) is used to construct the interface. 
The CMake build system will automatically download libigl and its dependencies using [CMake FetchContent](https://cmake.org/cmake/help/latest/module/FetchContent.html), thus requiring no setup on your part.

## Compile

Compile this project using the standard cmake routine:

```
    mkdir build
    cd build
    cmake ../
    make
    ./SKELETON_BASED_ARAP
```
