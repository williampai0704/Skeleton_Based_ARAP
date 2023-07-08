#include "Eigen.h"

/* Compute local rigidity energy
 * N : Neighbors of all vertices
 * Vi : Mesh before rotation
 * Vi_p : Mesh after rotation
 * R : Rotation for all cells
 *
 * Out : Local rigidity energy
*/

double compute_arap_energy(const Eigen::MatrixXd& Vi, const MatrixXd Vi_p, const std::vector<MatrixXd>& R, const std::vector<std::list<int>>& N)
{
    double energy = 0;
    for (int i = 0; i < Vi.rows(); i++)
    {
        VectorXd vi = Vi.row(i);
        VectorXd vi_p = Vi_p.row(i);
        MatrixXd Ri = R[i];
        std::list<int> v_n = N[i];
        for (std::list<int>::iterator it = v_n.begin(); it != v_n.end(); it++)
        {
            VectorXd ni = Vi.row(*it);
            VectorXd ni_p = Vi_p.row(*it);
            energy +=  pow(((vi - ni) - Ri * (vi_p - ni_p)).norm(), 2);
        }
    }
    return energy;
}