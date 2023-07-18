#pragma once
#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.h"
#include "ARAPInitEnum.h"

using namespace Eigen;

MatrixXd laplacian_init(const Mesh& mesh);
std::pair<bool, Vector3d> isConstraint(const std::vector<ControlPoint>& C, int index);
