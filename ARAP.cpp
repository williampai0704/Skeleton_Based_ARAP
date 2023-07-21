#include "Mesh.h"
#include "ARAPInitEnum.h"
using namespace Eigen;

#define tol 1e-3
#define alpha 25.0

std::pair<bool, Vector3d> isConstrained(const std::vector<ControlPoint>& C, int index) {

    for (const ControlPoint& c : C) {
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

double compute_reg_energy(const MatrixXd& W,
                          const MatrixXd& Vi, 
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
             // Weight of the edge
            double wij = W(i, *it);
            energy +=  (float)wij * pow(((vi - ni) - Ri * (vi_p - ni_p)).norm(), 2);
        }
    }
    return energy;
}

double compute_fit_energy(const MatrixXd& Vi, const std::vector<ControlPoint>& C)
{
    double energy = 0;
    for (int i = 0; i < Vi.rows(); i++)
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

double compute_total_energy(const MatrixXd& W,
                            const MatrixXd& Vi, 
                            const MatrixXd Vi_p, 
                            const std::vector<MatrixXd>& R, 
                            const std::vector<std::list<int>>& N,
                            const std::vector<ControlPoint>& C)
{
    double total_energy  = 0;
    total_energy += compute_reg_energy(W, Vi, Vi_p, R, N);
    total_energy += compute_fit_energy(Vi, C);
    return total_energy;
}

MatrixXd compute_covariance_matrix(const MatrixXd& W,
                                   const MatrixXd& Vi, 
                                   const MatrixXd& Vi_p, 
                                   const std::vector<std::list<int>>& N, 
                                   int& index)
{
    MatrixXd Si(Vi.cols(),Vi.cols());
    // Retrieve neighbors of v
    std::list<int> n = N[index];

    MatrixXd P = MatrixXd::Zero(Vi.cols(),n.size());
    MatrixXd P_p = MatrixXd::Zero(Vi.cols(),n.size());
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


MatrixXd compute_b(const MatrixXd& W, 
                   const std::vector<std::list<int>>& N, 
                   const MatrixXd& V, 
                   const std::vector<MatrixXd>& R, 
                   const std::vector<ControlPoint>& C)
{
    MatrixXd b = MatrixXd::Zero(V.rows(),V.cols());
    for (int i = 0; i < V.rows(); i++)
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

MatrixXd compute_Ri(const MatrixXd& W, 
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

/* Do the Laplacian initialization
 * mesh : Initial mesh
 *
 * Out : Updated mesh
 */
MatrixXd laplacian_init(const Mesh& mesh) {
    #define V mesh.V
    #define W mesh.W

    // Get control points
    const std::vector<ControlPoint>& C = mesh.getControlPoints();

    // Init L
    MatrixXd laplacian = -W;

    // Add diagonal value
    for (int i = 0; i < laplacian.rows(); i++) {
        laplacian(i, i) += -laplacian.row(i).sum();
    }

    // We want to solve A'Ax = A'(-By + weight_ * p), A and B being a part of L, y the constraint vertices and p the initial vertices

    // Build A, B and y
    MatrixXd A = MatrixXd::Zero(V.rows(), V.rows() - C.size());
    MatrixXd B = MatrixXd::Zero(V.rows(), C.size());
    MatrixXd y = MatrixXd::Zero(C.size(), V.cols());

    int a = 0;
    int b = 0;
    for (int i = 0; i < V.rows(); i++) {
        std::pair<bool, Vector3d> constraint = isConstrained(C, i);

        // B
        if (constraint.first) {
            B.col(b) = laplacian.col(i);
            y.row(b) = constraint.second;
            b++;
        }
        // A
        else {
            A.col(a) = laplacian.col(i);
            a++;
        }
    }

    // Build A' * A
    MatrixXd left = A.transpose() * A;

    // Build A' * (-By + L * p)
    MatrixXd right = A.transpose() * (-B * y + laplacian * V);

    // Solve
    MatrixXd x = left.ldlt().solve(right);

    // Get the result matrix
    MatrixXd new_V = MatrixXd::Zero(V.rows(), V.cols());
    a = 0;
    for (int i = 0; i < V.rows(); i++) {
        std::pair<bool, Vector3d> constraint = isConstrained(C, i);

        if (constraint.first) {
            new_V.row(i) = constraint.second;
        }
        else {
            new_V.row(i) = x.row(a);
            a++;
        }
    }

    return new_V;
    #undef V
    #undef W
}

MatrixXd arap(const Mesh& mesh, const int& kmax, const EInitialisationType& init, int* outInterationNumber = nullptr, float* outInitialEnergy = nullptr, float* outFinalEnergy = nullptr)
{
    #define V mesh.V
    #define N mesh.N
    #define W mesh.W

    // Get control points
    const std::vector<ControlPoint>&  C = mesh.getControlPoints();

    // Update L
    MatrixXd L = mesh.getL_withCP();

    // Init mesh before rotation
    MatrixXd previous_V = V;

    // Make an intial guess of new_V according to the initialisation type choosen:
    MatrixXd new_V;
    // User interaction
    if (init == EInitialisationType::e_LastFrame) {
        new_V = V;
        //std::cout << "Initiated with last frame" << std::endl;
    }
    // Laplacian initialization
    else if (init == EInitialisationType::e_Laplace) {
        new_V = laplacian_init(mesh);
        //std::cout << "Initiated with laplacian" << std::endl;
    }

    // Initialize energies
    float old_energy = 0;
    float new_energy = 0;

    int k = 0;
    do {
        // Find optimal Ri for each cell
        std::vector<MatrixXd> R(V.rows()); // Matrix of local rotations
        for (int i = 0; i < V.rows(); i++) {

            // Compute Ri
            MatrixXd Ri = compute_Ri(W, previous_V, new_V, N, i);

            // Store Ri
            R[i] = Ri;
        }
        // Compute right-hand side
        MatrixXd b = compute_b(W, N, previous_V, R, C);

        // Update mesh before rotation
        previous_V = new_V;

        // Find new optimal mesh
        new_V = L.ldlt().solve(b);

        // Update energies
        old_energy = new_energy;
        new_energy = compute_total_energy(W, previous_V, new_V, R, N, C);

        if (k == 0 && outInitialEnergy != nullptr)
            *outInitialEnergy = new_energy;

        k++;

    } while (k < kmax && abs(old_energy - new_energy) > tol);

    if (outInterationNumber != nullptr)
        *outInterationNumber = k;
    if (outFinalEnergy != nullptr)
        *outFinalEnergy = new_energy;

    return new_V;
    #undef V
    #undef N
    #undef W
}
