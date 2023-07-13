#include "Eigen.h"
#include "Mesh.h"
using namespace Eigen;


std::pair<bool, Vector3d> isConstrained(const std::vector<HandlePoint>& C, int index) {

    for (const HandlePoint& c : C) {
        if (index == c.vertexIndexInMesh) {
            return std::pair<bool, Vector3d>(true, c.wantedVertexPosition);
        }
    }
    return std::pair<bool, Vector3d>(false, Vector3d(0,0,0));
}

/* Compute local rigidity energy
 * N : Neighbors of all vertices
 * Vi : Mesh before rotation
 * Vi_p : Mesh after rotation
 * R : Rotation for all cells
 *
 * Out : Local rigidity energy
*/

double compute_reg_energy(const Eigen::MatrixXd& Vi, 
                          const MatrixXd Vi_p, 
                          const std::vector<MatrixXd>& R, 
                          const std::vector<std::list<int>>& N)
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

double compute_fit_energy(const Eigen::MatrixXd& Vi, const std::vector<HandlePoint>& C, const Eigen::MatrixXd& T)
{
    double energy = 0;
    for (int i = 0; i <= Vi.rows(); i++)
    {
        std::pair<bool,Vector3d> constrained = isConstrained(C, i);
        if (constrained.first)
        {
            VectorXd vi = Vi.row(i);
            VectorXd ti = (VectorXd) constrained.second;
            energy += pow((vi-ti).norm(), 2);
        }
    }
    return energy;
}

double compute_total_energy(const Eigen::MatrixXd& Vi, 
                            const MatrixXd Vi_p, 
                            const std::vector<MatrixXd>& R, 
                            const std::vector<std::list<int>>& N,
                            const std::vector<HandlePoint>& C, 
                            const Eigen::MatrixXd& T)
{
    double total_energy  = 0;
    total_energy += compute_reg_energy(Vi, Vi_p, R, N);
    total_energy += compute_fit_energy(Vi, C, T);
    return total_energy;
}

MatrixXd compute_covariance_matrix(const Eigen::MatrixXd& W, 
                                   const MatrixXd& Vi, 
                                   const MatrixXd& Vi_p, 
                                   const std::vector<std::list<int>>& N, 
                                   int& index)
{
    MatrixXd Si(Vi.cols(),Vi.cols());
    // Retrieve neighbors of v
    std::list<int> n = N[index];

    MatrixXd P = MatrixXd::Zero(Vi.rows(),n.size());
    MatrixXd P_p = MatrixXd::Zero(Vi.rows(),n.size());
    DiagonalMatrix<double,Eigen::Dynamic> D(n.size());

    Vector3d vi = Vi.row(index);
    Vector3d vi_p = Vi_p.row(index);

    int k = 0;
    for(std::list<int>::iterator j = n.begin(); j != n.end(); j++, k++)
    {
        Vector3d ni = Vi.row(*j);
        P.col(k) = vi-ni;

        Vector3d ni_p = Vi_p.row(*j);
        P_p.col(k) = vi_p - ni_p;

        D.diagonal()[k] = W(index, *j);
    }
    Si = P * D * P_p.transpose();
    return Si;
}


MatrixXd compute_b(const Eigen::MatrixXd& W, 
                   const std::vector<std::list<int>>& N, 
                   const MatrixXd& V, 
                   const std::vector<MatrixXd>& R, 
                   const std::vector<HandlePoint>& C)
{
    MatrixXd b = MatrixXd::Zero(V.rows(),V.rows());
    for (int i = 0; i <= V.rows(); i++)
    {
        VectorXd vi = V.row(i);
        std::list<int> n = N[i];
        MatrixXd Ri = R[i];
        std::pair<bool,Vector3d> constrained = isConstrained(C, i);
        if (constrained.first)
        {
            b.row(i) = (VectorXd) constrained.second;
        }
        else
        {
            // For each neighbor
            for (std::list<int>::iterator it = n.begin(); it != n.end(); ++it) 
            {
                std::pair<bool, Vector3d> constrained_n = isConstrained(C, *it);

                VectorXd neighbor = V.row(*it);
                
                // Compensate the term of the left part
                if (constrained_n.first) {
                    b.row(i) += (double)W(i, *it) * constrained_n.second;
                }

                // Weight of the edge
                double wij = W(i, *it);

                // Neighbor Rotation matrix
                MatrixXd Rj = R[*it];

                // Add the term
                b.row(i) += (double)wij / 2 * (Ri + Rj) * (vi - neighbor);

            }
        }

    }
    return b; 
}

MatrixXd compute_Ri(const Eigen::MatrixXd& W, 
                   const MatrixXd& Vi, 
                   const MatrixXd& Vi_p, 
                   const std::vector<std::list<int>>& N, 
                   int& i)
{
    MatrixXd Si = compute_covariance_matrix(W, Vi, Vi_p, N, i);

    JacobiSVD<MatrixXd> svd(Si, ComputeThinU | ComputeThinV);

    // Compute rotation Ri
    DiagonalMatrix<double, 3> D(1, 1, (svd.matrixV() * svd.matrixU().transpose()).determinant());
    MatrixXd Ri = svd.matrixV() * D * svd.matrixU().transpose();
    return Ri;
}
