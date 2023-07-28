# Skeleton-Based ARAP

This project implemented a skeleton-based ARAP (As Rigid As Possible) deformation algorithm on animal 3D mesh.

## Dependencies
This project uses [Eigen3](https://gitlab.com/libeigen/eigen) library in version 3.4.0. 
Mac users can also install by 
```
brew install eigen
```
remember to change the Eigen3_DIR in CMakeList.txt to corresponding path.

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

## Files

### Mesh.cpp

Defines the object of a `Mesh()`.

#### Variables

`Eigen::MatrixXd V`: vertices positions.

`Eigen::MatrixXd F`: faces, defined by vertices' indexes.

`Eigen::MatrixXd W`:

`Eigen::MatrixXd L`:

`std::vector<ControlPoint> C`: list of control points.

#### Functions



### InterfaceManager.cpp

### ARAP.cpp

### main.cpp

The main function to run the project.

`read_pseudo_off()`: read pseudo mesh as `.off` file. Output: a tuple of its vertices and faces.

`read_surface_off()`: read surface mesh as `.off` file. Output: a tuple of its vertices and faces.

`get_bone()`: read the matching points of the skeleton bone on its pseudo mesh.

`read_attachment()`: read the coefficients for Linear Blend Skinning.

