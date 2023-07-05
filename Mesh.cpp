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