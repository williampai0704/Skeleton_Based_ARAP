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

`std::vector<std::list<int>> N`: neighbours of all vertices.

`Eigen::MatrixXd W`: weight matrix.

`Eigen::MatrixXd L`: Laplacian matrix.

`std::vector<ControlPoint> C`: list of control points.

#### Functions

`void computeL_W_N()`: compute and store the mesh's neighbour vertices `N`, weight matrix `W` and Laplacian matrix `L` continually.

`Eigen::MatrixXd getL_withCP() const`: modify the Laplacian matrix `L` in accordance with control points `C`, and return it.

`void addControlPoint(int vertexIndex)`: push back the selected vertex into the list of control points `C`.

### ControlPoint.h

Defines the object of a `ControlPoint()`.

#### Variables

`int vertexIndexInMesh`: vertex index in the mesh.

`Eigen::RowVector3d wantedVertexPosition`: the position of the control point.

### InterfaceManager.cpp

### ARAP.cpp

### main.cpp

The main function to run the project.

`tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_pseudo_off()`: read pseudo mesh as `.off` file.  
Output: a tuple of its vertices and faces.

`tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_surface_off()`: read surface mesh as `.off` file.  
Output: a tuple of its vertices and faces.

`tuple<Eigen::MatrixXd, Eigen::MatrixXd> get_bone(Eigen::MatrixXd V)`: read the matching points of the skeleton bone on its pseudo mesh.  
Input: pseudo mesh vertices.  
Output: a tuple of skeleton vertices and the matching table.

`Eigen::MatrixXd read_attachment()`: read and return the coefficients for Linear Blend Skinning.

