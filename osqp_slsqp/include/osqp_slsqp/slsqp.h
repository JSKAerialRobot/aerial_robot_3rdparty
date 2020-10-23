#pragma once

#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>
#include <cmath>
#include <unistd.h>
#include <exception>
#include <stdexcept>
#include <iostream>

template<typename T1, typename T2, typename T3>
class SLSQPSolver {
public:
  SLSQPSolver(int n_variables, int n_constraints, T1 cost_function_gradient, T2 inequality_constraint, T3 inequality_constraint_gradient, Eigen::VectorXd lower_bound, Eigen::VectorXd upper_bound, Eigen::VectorXd initial_x, double sqp_relative_tolerance = 1e-4, double qp_relative_tolerance = 1e-2, int max_loop_count = 1000, bool verbose = false) :
    n_variables_(n_variables),
    n_constraints_(n_constraints),
    cost_function_gradient_(cost_function_gradient),
    inequality_constraint_(inequality_constraint),
    inequality_constraint_gradient_(inequality_constraint_gradient),
    lower_bound_(lower_bound),
    upper_bound_(upper_bound),
    initial_x_(initial_x),
    sqp_relative_tolerance_(sqp_relative_tolerance),
    qp_relative_tolerance_(qp_relative_tolerance),
    max_loop_count_(max_loop_count),
    verbose_(verbose),
    hessian_first_update_(true),
    qp_first_solve_(true)
  {
    qp_hessian_ = Eigen::MatrixXd::Identity(n_variables_, n_variables_);
  }

  void solve()
  {

    Eigen::VectorXd x = initial_x_;
    initSolver(x);
    for (int i = 0; i < max_loop_count_; ++i) {
      x_log_.push_back(x);
      Eigen::VectorXd solution = QP(x);
      x += solution;

      if (solution.norm() < sqp_relative_tolerance_) {
        break;
      }
    }
  }

  Eigen::VectorXd getLastX() {
    return x_log_.back();
  }

  std::vector<Eigen::VectorXd> getXLog() {
    return x_log_;
  }


private:
  int n_variables_;
  int n_constraints_;
  T1 cost_function_gradient_;
  T2 inequality_constraint_;
  T3 inequality_constraint_gradient_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;
  Eigen::VectorXd initial_x_;
  double sqp_relative_tolerance_;
  double qp_relative_tolerance_;
  int max_loop_count_;
  bool verbose_;
  bool hessian_first_update_;
  bool qp_first_solve_;

  OsqpEigen::Solver qp_solver_;
  Eigen::MatrixXd qp_hessian_;
  Eigen::SparseMatrix<double> qp_hessian_sparse_;
  Eigen::VectorXd qp_gradient_;
  Eigen::MatrixXd qp_linear_matrix_;
  Eigen::SparseMatrix<double> qp_linear_matrix_sparse_;
  Eigen::VectorXd qp_lower_bound_;
  Eigen::VectorXd qp_upper_bound_;
  Eigen::VectorXd qp_x_;
  Eigen::VectorXd qp_dual_solution_;
  std::vector<Eigen::VectorXd> x_log_;

  double autoScale(const Eigen::VectorXd& s, const Eigen::VectorXd& q)
  {
    double s_norm2 = s.transpose() * s;
    double y_norm2 = q.transpose() * q;
    double ys = std::abs(q.transpose() * s);
    if (ys == 0.0 || y_norm2 == 0 || s_norm2 == 0) {
      return 1.0;
    } else {
      return y_norm2 / ys;
    }
  }

  Eigen::MatrixXd updateHessianBFGS(Eigen::MatrixXd h_prev, const Eigen::VectorXd& x, const Eigen::VectorXd& x_prev, const Eigen::VectorXd& lambda_prev, const Eigen::VectorXd& grad_f, const Eigen::VectorXd& grad_f_prev, const Eigen::MatrixXd& grad_g, const Eigen::MatrixXd& grad_g_prev)
  {
    Eigen::VectorXd s = x - x_prev;
    Eigen::VectorXd q = (grad_f - grad_f_prev) + (grad_g - grad_g_prev).transpose() * lambda_prev;

    double qs = q.transpose() * s;
    Eigen::VectorXd Hs = h_prev * s;
    double sHs = s.transpose() * h_prev * s;

    if (sHs < 0.0 || hessian_first_update_) {
      h_prev = Eigen::MatrixXd::Identity(n_variables_, n_variables_) * autoScale(s, q);
      Hs = h_prev * s;
      sHs = s.transpose() * h_prev * s;
      hessian_first_update_ = false;
    }

    if (qs < 0.2 * sHs) {
      double update_factor = (1 - 0.2) / (1 - qs / sHs);
      q = update_factor * q + (1 - update_factor) * Hs;
      qs = q.transpose() * s;
    }
    auto h = h_prev + (q * q.transpose()) / qs - (Hs * Hs.transpose()) / sHs;

    return h;
  }

  Eigen::SparseMatrix<double> convertSparseMatrix(const Eigen::MatrixXd& mat)
  {
    Eigen::SparseMatrix<double> ret;
    ret.resize(mat.rows(), mat.cols());
    for (size_t i = 0; i < ret.rows(); ++i) {
      for (size_t j = 0; j < ret.cols(); ++j) {
        if (mat(i, j) != 0.0) {
          ret.insert(i, j) = mat(i, j);
        }
      }
    }
    return ret;
  }

  void initSolver(const Eigen::VectorXd& x)
  {
    qp_solver_.settings()->setWarmStart(true);
    qp_solver_.settings()->setVerbosity(verbose_);
    qp_solver_.settings()->setRelativeTolerance(qp_relative_tolerance_);

    qp_hessian_sparse_ = qp_hessian_.sparseView();
    qp_gradient_ = cost_function_gradient_(x); //q
    qp_linear_matrix_ = inequality_constraint_gradient_(x);
    qp_linear_matrix_sparse_ = convertSparseMatrix(qp_linear_matrix_);
    auto inequality_constraint = inequality_constraint_(x);
    qp_lower_bound_ = lower_bound_ - inequality_constraint; //lb
    qp_upper_bound_ = upper_bound_ - inequality_constraint; //ub

    qp_solver_.data()->setNumberOfVariables(n_variables_);
    qp_solver_.data()->setNumberOfConstraints(n_constraints_);
    bool ok = true;
    ok &= qp_solver_.data()->setHessianMatrix(qp_hessian_sparse_);
    ok &= qp_solver_.data()->setGradient(qp_gradient_);
    ok &= qp_solver_.data()->setLinearConstraintsMatrix(qp_linear_matrix_sparse_);
    ok &= qp_solver_.data()->setLowerBound(qp_lower_bound_);
    ok &= qp_solver_.data()->setUpperBound(qp_upper_bound_);

    if (!ok) {
      std::cerr << "variable : " << x.transpose() << std::endl;
      throw std::runtime_error("initSolver failed");
    }
    qp_solver_.initSolver();
  }

  Eigen::VectorXd QP(const Eigen::VectorXd& x)
  {
    //update solver
    Eigen::VectorXd qp_gradient_prev_ = qp_gradient_;
    Eigen::MatrixXd qp_linear_matrix_prev_ = qp_linear_matrix_;

    qp_gradient_ = cost_function_gradient_(x); //q
    qp_linear_matrix_ = inequality_constraint_gradient_(x); //A
    auto inequality_constraint = inequality_constraint_(x);
    qp_lower_bound_ = lower_bound_ - inequality_constraint; //lb
    qp_upper_bound_ = upper_bound_ - inequality_constraint; //ub

    if (qp_first_solve_) {
      qp_first_solve_ = false;
    } else {
      qp_hessian_ = updateHessianBFGS(qp_hessian_, x, qp_x_, qp_dual_solution_, qp_gradient_, qp_gradient_prev_, qp_linear_matrix_, qp_linear_matrix_prev_);
    }

    qp_hessian_sparse_ = convertSparseMatrix(qp_hessian_);
    qp_linear_matrix_sparse_ = convertSparseMatrix(qp_linear_matrix_);

    bool ok = true;
    ok &= qp_solver_.updateHessianMatrix(qp_hessian_sparse_); //H
    ok &= qp_solver_.updateGradient(qp_gradient_); //q
    ok &= qp_solver_.updateLinearConstraintsMatrix(qp_linear_matrix_sparse_); //A
    ok &= qp_solver_.updateBounds(qp_lower_bound_, qp_upper_bound_);
    ok &= qp_solver_.solve();

    if (!ok) {
      std::cerr << "variable : " << x.transpose() << std::endl;
      std::cerr << "solution : " << qp_solver_.getSolution().transpose() << std::endl;
      throw std::runtime_error("QP failed");
    }

    qp_dual_solution_ = qp_solver_.getDualSolution();
    qp_x_ = x;

    return qp_solver_.getSolution();
  }
};

//helper

template <typename T1, typename T2, typename T3>
SLSQPSolver<T1, T2, T3> makeSLSQPSolver(int n_variables, int n_constraints, T1 cost_function_gradient, T2 inequality_constraint, T3 inequality_constraint_gradient, Eigen::VectorXd lower_bound, Eigen::VectorXd upper_bound, Eigen::VectorXd initial_x, double sqp_relative_tolerance = 1e-4, double qp_relative_tolerance = 1e-2, int max_loop_count = 1000, bool verbose = false)
{
  return SLSQPSolver<T1, T2, T3>(n_variables, n_constraints, cost_function_gradient, inequality_constraint, inequality_constraint_gradient, lower_bound, upper_bound, initial_x, sqp_relative_tolerance, qp_relative_tolerance);
}
