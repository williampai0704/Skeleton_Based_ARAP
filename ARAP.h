#ifndef ARAP_H
#define ARAP_H

#include "Eigen.h"
#include "Mesh.h"
#include "ARAPInitEnum.h"

using namespace Eigen;

std::pair<bool, Vector3d> isConstrained(const std::vector<ControlPoint>& C, int index);
double compute_reg_energy(const Eigen::MatrixXd& W, const Eigen::MatrixXd& Vi, const MatrixXd Vi_p, const std::vector<MatrixXd>& R, const std::vector<std::list<int>>& N);
double compute_fit_energy(const Eigen::MatrixXd& Vi, const std::vector<ControlPoint>& C);
double compute_total_energy(const Eigen::MatrixXd& W, const Eigen::MatrixXd& Vi, const MatrixXd Vi_p, const std::vector<MatrixXd>& R, const std::vector<std::list<int>>& N, const std::vector<ControlPoint>& C);
MatrixXd compute_covariance_matrix(const Eigen::MatrixXd& W, const MatrixXd& Vi, const MatrixXd& Vi_p, const std::vector<std::list<int>>& N, int& index);
MatrixXd compute_b(const Eigen::MatrixXd& W, const std::vector<std::list<int>>& N, const MatrixXd& V, const std::vector<MatrixXd>& R, const std::vector<ControlPoint>& C);
MatrixXd compute_Ri(const Eigen::MatrixXd& W, const MatrixXd& Vi, const MatrixXd& Vi_p, const std::vector<std::list<int>>& N, int& i);
MatrixXd laplacian_init(const Mesh& mesh);
MatrixXd arap(const Mesh& mesh, const int& kmax, const EInitialisationType& init, int* outInterationNumber = nullptr, float* outInitialEnergy = nullptr, float* outFinalEnergy = nullptr);

#endif
