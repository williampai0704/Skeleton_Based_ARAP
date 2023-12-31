#include "Mesh.h"
#include <iostream>

void Mesh::InitMesh(MeshType type)
{
	switch (type)
	{
	case MeshType::CUBE:
		Mesh::V = (Eigen::MatrixXd(8, 3) << 0.0, 0.0, 0.0,
				   0.0, 0.0, 1.0,
				   0.0, 1.0, 0.0,
				   0.0, 1.0, 1.0,
				   1.0, 0.0, 0.0,
				   1.0, 0.0, 1.0,
				   1.0, 1.0, 0.0,
				   1.0, 1.0, 1.0)
					  .finished();
		Mesh::F = (Eigen::MatrixXi(12, 3) << 1, 7, 5,
				   1, 3, 7,
				   1, 4, 3,
				   1, 2, 4,
				   3, 8, 7,
				   3, 4, 8,
				   5, 7, 8,
				   5, 8, 6,
				   1, 5, 6,
				   1, 6, 2,
				   2, 6, 8,
				   2, 8, 4)
					  .finished()
					  .array() -
				  1;
		break;

	case MeshType::SKELETON:
		Mesh::V = (Eigen::MatrixXd(15, 3) << 0.0, 0.0, 0.0,
				   1.0, 0.0, 0.0,
				   0.0, 1.0, 0.0,
				   0.0, -1.0, 0.0,
				   -1.0, 0.0, 0.0,
				   0.0, 0.0, 1.0,
				   1.0, 0.0, 1.0,
				   0.0, 1.0, 1.0,
				   0.0, -1.0, 1.0,
				   -1.0, 0.0, 1.0,
				   0.0, 0.0, 2.0,
				   1.0, 0.0, 2.0,
				   0.0, 1.0, 2.0,
				   0.0, -1.0, 2.0,
				   -1.0, 0.0, 2.0)
					  .finished();
		Mesh::F = (Eigen::MatrixXi(28, 3) << 1, 3, 2,
				   1, 5, 3,
				   1, 4, 5,
				   1, 2, 4,
				   6, 8, 7,
				   6, 10, 8,
				   6, 9, 10,
				   6, 7, 9,
				   11, 13, 12,
				   11, 15, 13,
				   11, 14, 15,
				   11, 12, 14,
				   2, 7, 4,
				   4, 7, 9,
				   4, 9, 5,
				   5, 9, 10,
				   3, 5, 10,
				   3, 10, 8,
				   3, 8, 7,
				   2, 3, 7,
				   7, 12, 9,
				   9, 12, 14,
				   9, 14, 10,
				   10, 14, 15,
				   8, 10, 15,
				   8, 15, 13,
				   8, 13, 12,
				   7, 8, 12)
					  .finished()
					  .array() -
				  1;
		break;
	default:
		std::cout << "please initial mesh with a deafult type or customized model" << std::endl;
		break;
	}
}

void Mesh::InitMesh(Eigen::MatrixXd V, Eigen::MatrixXi F)
{
}

#define eps 1e-10

Eigen::MatrixXd Mesh::getVerticesFromIndex(const std::vector<int> &indexes) const
{
	Eigen::MatrixXd verts = Eigen::MatrixXd::Zero(indexes.size(), 3);
	for (int i = 0; i < verts.rows(); i++)
		verts.row(i) = V.row(indexes[i]);
	return verts;
}

std::vector<ControlPoint *> Mesh::getControlPointsW()
{
	std::vector<ControlPoint *> cpp = std::vector<ControlPoint *>();
	cpp.reserve(C.size());
	for (auto &cp : C)
		cpp.push_back(&cp);
	return cpp;
}

std::vector<int> Mesh::getControlPointsIndex() const
{
	std::vector<int> indexes;
	indexes.reserve(C.size());
	for (const auto &cp : C)
		indexes.push_back(cp.vertexIndexInMesh);
	return indexes;
}

Eigen::MatrixXd Mesh::getControlPointsWantedPosition() const
{
	Eigen::MatrixXd cpPosition = Eigen::MatrixXd::Zero(C.size(), 3);
	for (int i = 0; i < cpPosition.rows(); i++)
		cpPosition.row(i) = C[i].wantedVertexPosition;
	return cpPosition;
}

Eigen::MatrixXd Mesh::getControlPointsWantedPositionBySelection(const std::vector<int> &selection, bool invert) const
{
	std::vector<const ControlPoint *> cpToUse = std::vector<const ControlPoint *>();
	for (const auto &cp : C)
	{
		bool inSelection = false;
		for (const auto &i : selection)
			if (i == cp.vertexIndexInMesh)
			{
				inSelection = true;
				break;
			}

		if (inSelection ^ invert)
			cpToUse.push_back(&cp);
	}

	Eigen::MatrixXd wantedPositions = Eigen::MatrixXd::Zero(cpToUse.size(), 3);
	for (int i = 0; i < cpToUse.size(); i++)
		wantedPositions.row(i) = cpToUse[i]->wantedVertexPosition;

	return wantedPositions;
}

bool Mesh::isAControlPoint(int vertexIndex) const
{
	for (const auto &cp : C)
		if (cp.vertexIndexInMesh == vertexIndex)
			return true;
	return false;
}

ControlPoint *Mesh::getControlPoint(int vertexIndex)
{
	for (auto &cp : C)
		if (cp.vertexIndexInMesh == vertexIndex)
			return &cp;
	return nullptr;
}

int Mesh::getControlPointCount() const
{
	return C.size();
}

void Mesh::addControlPoint(int vertexIndex)
{
	if (vertexIndex < 0 || vertexIndex >= V.rows() || isAControlPoint(vertexIndex))
		return;
	C.push_back(ControlPoint(vertexIndex, V.row(vertexIndex)));
}

void Mesh::addControlPoint(int vertexIndex, Eigen::RowVector3d position)
{
	if (vertexIndex < 0 || vertexIndex >= V.rows())
		return;

	// If already exists, change its control position to position
	for (auto &cp : C)
		if (cp.vertexIndexInMesh == vertexIndex)
		{
			cp.wantedVertexPosition = position;
			return;
		}

	// Else just add it
	C.push_back(ControlPoint(vertexIndex, position));
}

void Mesh::removeControlPoint(int vertexIndex)
{
	if (vertexIndex < 0 || vertexIndex >= V.rows())
		return;

	int index = -1;
	for (int i = 0; i < C.size(); i++)
		if (C[i].vertexIndexInMesh == vertexIndex)
		{
			index = i;
			break;
		}

	if (index != -1)
		C.erase(std::next(C.begin(), index));
}

void Mesh::printControlPoints() const
{
	std::cout << "Control Points:\n";
	for (auto &cp : C)
	{
		std::cout << cp.vertexIndexInMesh << " : " << V.row(cp.vertexIndexInMesh) << " -> " << cp.wantedVertexPosition << "\n";
	}
	std::cout << std::endl;
}

/* Find one-ring neighbors of all the vertices in V.
 * V : Matrix of vertices
 * F : Matrix of faces
 *
 * Out : Vector of list of neigbors indices, set in the mesh's N attribute
 */
void Mesh::computeN()
{
	N = std::vector<std::list<int>>(V.rows());

	for (int i = 0; i < F.rows(); i++)
	{
		for (int j = 0; j < F.cols(); j++)
		{
			N[F(i, j)].push_back(F(i, (j + 1) % F.cols()));
			N[F(i, j)].push_back(F(i, (j + 2) % F.cols()));
		}
	}

	for (int i = 0; i < V.rows(); i++)
	{
		N[i].sort();
		N[i].unique();
	}
}
/* Compute weights wij
 * V : Matrix of vertices
 * F : Matrix of faces
 *
 * Out : Weights wij = 1/2 * (cot(aij) + cot(bij)), in mesh's W attribute
 */
void Mesh::computeW()
{
	W = Eigen::MatrixXd::Zero(V.rows(), V.rows());

	for (int i = 0; i < F.rows(); i++)
	{
		// Compute edges vectors CCW
		Eigen::Vector3d v1 = V.row(F(i, 1)) - V.row(F(i, 0));
		Eigen::Vector3d v2 = V.row(F(i, 2)) - V.row(F(i, 1));
		Eigen::Vector3d v3 = V.row(F(i, 0)) - V.row(F(i, 2));

		// Compute the angles
		double a201 = acos(v1.dot(-v3) / (v1.norm() * v3.norm()));
		double a012 = acos(-v1.dot(v2) / (v1.norm() * v2.norm()));
		double a120 = acos(-v2.dot(v3) / (v2.norm() * v3.norm()));

		// Add the angles
		W(F(i, 0), F(i, 1)) += cos(a120) / sin(a120);
		W(F(i, 1), F(i, 0)) += cos(a120) / sin(a120);

		W(F(i, 1), F(i, 2)) += cos(a201) / sin(a201);
		W(F(i, 2), F(i, 1)) += cos(a201) / sin(a201);

		W(F(i, 2), F(i, 0)) += cos(a012) / sin(a012);
		W(F(i, 0), F(i, 2)) += cos(a012) / sin(a012);
	}

	// Divide all the weights by 2
	W = (float)1 / 2 * W;

	// Put small values to 0
	for (int i = 0; i < W.rows(); i++)
	{
		for (int j = 0; j < W.cols(); j++)
		{
			if (W(i, j) < eps)
			{
				W(i, j) = 0;
			}
		}
	}
}
void Mesh::computeL()
{
	L = -W;

	// Add diagonal value
	for (int i = 0; i < L.rows(); i++)
	{
		L(i, i) += -L.row(i).sum();
	}
}
void Mesh::computeL_W_N()
{
	computeN();
	computeW();
	computeL();
}

Eigen::MatrixXd Mesh::getL_withCP() const
{
	Eigen::MatrixXd _L = L;

	for (const ControlPoint &c : C)
	{
		int index = c.vertexIndexInMesh;
		_L.row(index) = Eigen::VectorXd::Zero(_L.cols());
		_L.col(index) = Eigen::VectorXd::Zero(_L.rows());
		_L(index, index) = 1;
	}

	return _L;
}

// void Mesh::updateVertices(const Eigen::MatrixXd &newVertices, const std::vector<int> &vertexIndices)
// {
// 	for (std::vector<int>::const_iterator it = vertexIndices.begin(); it != vertexIndices.end(); ++it)
// 	{
// 		int i = *it;
// 		if (i >= 0 && i < V.rows())
// 		{
// 			V.row(i) = newVertices.row(i);
// 		}
// 	}
// }