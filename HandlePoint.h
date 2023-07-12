#include "Eigen.h"
using namespace Eigen;

struct HandlePoint
{
	int					vertexIndexInMesh;
	Eigen::RowVector3d	wantedVertexPosition;

	HandlePoint(int index, Eigen::RowVector3d position) : vertexIndexInMesh(index), wantedVertexPosition(position) {}
};