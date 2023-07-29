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
## Branch
"main" branch perform ARAP with bear surface mesh.
For the ARAP deformation with only pseudo mesh, checkout "dev_GUI" branch.

## Files

### Input OFF Files

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

`void addControlPoint(int vertexIndex)`: push back the selected vertex into the list of control points `C`. Input as vertex index.

`void removeControlPoint(int vertexIndex)`: remove the selected vertex from the list of control points `C`. Input as vertex index.

### ControlPoint.h

Defines the object of a `ControlPoint()`.

#### Variables

`int vertexIndexInMesh`: vertex index in the mesh.

`Eigen::RowVector3d wantedVertexPosition`: the position of the control point.

### ARAP.cpp

The core of ARAP implementation.

#### Functions

`MatrixXd arap(const Mesh &mesh, const int &kmax, const EInitialisationType &init, int *outInterationNumber = nullptr, float *outInitialEnergy = nullptr, float *outFinalEnergy = nullptr)`: apply ARAP algorithm on the input mesh `mesh` within the given `kmax` loops.  
**Output:** the updated mesh vertices 

### InterfaceManager.cpp

The control of interface actions.

#### Variables

`bool isBone`: toggle the visualization of the surface mesh and skeleton vertices.

`std::vector<int> selection`: list of selected points.

`bool mouseIsPressed`: record whether the mouse is currently pressed or not.

`Eigen::Vector3d lastProjectedPoint`: stores the last projected point by the mouse.

`Eigen::Vector3d firstMoveDirection`: define the movement of the point on the x, y, or z-axis.

`bool moveSelectionMode`: toggles the mouse movement mode into mode 0 (move the camera) and mode 1 (move the selected point).

#### Functions

`void displaySelectedPoints(igl::opengl::glfw::Viewer &viewer, const Mesh &mesh, const Mesh &bone) const`: display the selected points, control points and skeleton vertices onto the interface viewer.

`void projectOnMoveDirection(igl::opengl::glfw::Viewer &viewer, Eigen::Vector3d &projectionReceiver) const`: project the movement of the point on the defined axis `firstMoveDirection`, and stores it in `projectionReceiver`.

`void onMousePressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, bool isShiftPressed, Eigen::MatrixXd bone_index)`: selects the closest vertex on the skeleton to the mouse when the user presses the mouse.

`void onMouseReleased()`: toggle `mouseIsPressed` back to false.

`bool onMouseMoved(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, bool &needArap, const EInitialisationType &initialisationType)`: under `moveSelectionMode == 1`, move and update the selected point on the defined axis.

`void onKeyPressed(igl::opengl::glfw::Viewer &viewer, Mesh &mesh, Mesh &bone, Mesh &surface, unsigned char key, bool isShiftPressed, bool &needArap, EInitialisationType &initType)`: perform actions according to the input `key`.  
`key == 'M'`: toggle `moveSelectionMode`  
`key == 'C'`: add selected point as control point with `mesh.addControlPoint()`  
`key == 'R'`: remove the selected point from the list of control points with `mesh.removeControlPoint()`  
`key == 'X'`, `'Y'`, `'Z'`: toggle movement axis  
`key == 'B'`: toggle surface mesh and skeleton vertices visibility.

### main.cpp

The main function of the project.

#### Variables

`bool needToPerformArap`: toggle the gate to conduct ARAP deformation.

`igl::opengl::glfw::Viewer viewer`: initialize the interface viewer based on the [libigl](https://github.com/libigl/libigl) library.

#### Functions

`tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_pseudo_off()`: read pseudo mesh as `.off` file.  
**Output:** a tuple of its vertices and faces.

`tuple<Eigen::MatrixXd, Eigen::MatrixXi> read_surface_off()`: read surface mesh as `.off` file.  
**Output:** a tuple of its vertices and faces.

`tuple<Eigen::MatrixXd, Eigen::MatrixXd> get_bone(Eigen::MatrixXd V)`: read the matching points of the skeleton bone on its pseudo mesh.  
**Input:** pseudo mesh vertices.  
**Output:** a tuple of skeleton vertices and the matching table.

`Eigen::MatrixXd read_attachment()`: read and return the coefficients for Linear Blend Skinning.

`void performARAP(Mesh &pseudo_mesh, Mesh& bone, const EInitialisationType& initialisationType, igl::opengl::glfw::Viewer& viewer, const InterfaceManager& interfaceManager)`: conduct ARAP deformation on the pseudo mesh.
