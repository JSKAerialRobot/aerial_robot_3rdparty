#pragma once

#include <Eigen/Core>
#include <casadi/casadi.hpp>

Eigen::MatrixXd casadiDmToEigenMatrix(casadi::DM dm);
Eigen::VectorXd casadiDmToEigenVector(casadi::DM dm);
casadi::DM eigenVectorToCasadiDm(Eigen::VectorXd vec);
casadi::DM eigenMatrixToCasadiDM(Eigen::MatrixXd mat);
