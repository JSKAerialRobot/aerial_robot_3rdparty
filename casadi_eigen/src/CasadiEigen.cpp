#include <CasadiEigen/CasadiEigen.h>

Eigen::MatrixXd casadiDmToEigenMatrix(casadi::DM dm)
{
  size_t rows = dm.size1();
  size_t cols = dm.size2();
  Eigen::MatrixXd ret = Eigen::MatrixXd::Zero(rows, cols);
  std::memcpy(ret.data(), dm.ptr(), sizeof(double) * rows * cols);
  return ret;
}

Eigen::VectorXd casadiDmToEigenVector(casadi::DM dm)
{
  size_t rows = dm.size1();
  size_t cols = dm.size2();
  Eigen::VectorXd ret = Eigen::VectorXd::Zero(rows * cols);
  std::memcpy(ret.data(), dm.ptr(), sizeof(double) * rows * cols);
  return ret;
}

casadi::DM eigenVectorToCasadiDm(Eigen::VectorXd vec)
{
  size_t length = vec.size();
  casadi::DM ret = casadi::DM::zeros(length);
  std::memcpy(ret.ptr(), vec.data(), sizeof(double) * length);
  return ret;
}

casadi::DM eigenMatrixToCasadiDM(Eigen::MatrixXd mat)
{
  size_t rows = mat.rows();
  size_t cols = mat.cols();
  casadi::DM ret = casadi::DM::zeros(rows, cols);
  std::memcpy(ret.ptr(), mat.data(), sizeof(double) * rows * cols);
  return ret;
}

