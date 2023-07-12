#include <Eigen/Dense>
#include "HandlePoint.h"

struct Mesh
{
    enum MeshType
    {
        CUBE = 1,
        SKELETON = 2
    };

private:
public:
    Eigen::MatrixXd V; // Vertices positions
    Eigen::MatrixXi F; // Faces, defined by vertices' indexes
    void InitMesh(MeshType type);
    void InitMesh(Eigen::MatrixXd V, Eigen::MatrixXi F);
};
