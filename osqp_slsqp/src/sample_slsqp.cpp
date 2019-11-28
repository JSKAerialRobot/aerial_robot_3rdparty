#include <osqp_slsqp/slsqp.h>
#include <cmath>

/**********************************/
/*
The problem is from NLOPT tutorial

variable: x in R^2 (2D vector)

min sqrt(x_2)

s.t. x_2 >= 0
     x_2 >= (a1 x1 + b1)^3
     x_2 >= (a2 x1 + b2)^3

for a1 = 2, b1 = 0, a2 = -1, b2 = 1

*/
/**********************************/


class SampleSLSQP {

  int n_variables;
  int n_constraints;

  Eigen::VectorXd costFunctionGradient(const Eigen::VectorXd& x) {
    Eigen::VectorXd grad(n_variables);
    grad(0) = 0.0;
    grad(1) = 0.5 / std::sqrt(x(1));
    return grad;
  }

  double inequalityConstraint(const Eigen::VectorXd& x, double a, double b) {
    return x(1) - (a*x(0) + b) * (a*x(0) + b) * (a*x(0) + b);
  }

  Eigen::VectorXd inequalityConstraintGrad(const Eigen::VectorXd& x, double a, double b) {
    Eigen::VectorXd grad(n_variables);
    grad(0) = - 3 * a * (a * x(0) + b) * (a * x(0) + b);
    grad(1) = 1.0;
    return grad;
  }


public:
  SampleSLSQP() {
    n_variables = 2;
    n_constraints = 3;
  }

  void solve() {
    auto cost_function_gradient = [this](const Eigen::VectorXd& x) -> Eigen::VectorXd
    {
      return this->costFunctionGradient(x);
    };

    auto inequality_constraint = [this](const Eigen::VectorXd& x) -> Eigen::VectorXd
    {
      Eigen::VectorXd constraint(n_constraints);
      constraint << x(1), this->inequalityConstraint(x, 2.0, 0.0), this->inequalityConstraint(x, -1.0, 1.0);
      return constraint;
    };

    auto inequality_constraint_gradient = [this](const Eigen::VectorXd& x) -> Eigen::MatrixXd
    {
      Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(this->n_constraints, this->n_variables);
      Eigen::VectorXd x2_inequality_grad(this->n_variables);
      x2_inequality_grad << 0.0, 1.0;
      grad << x2_inequality_grad.transpose(), this->inequalityConstraintGrad(x, 2.0, 0.0).transpose(), this->inequalityConstraintGrad(x, -1.0, 1.0).transpose();
      return grad;
    };

    Eigen::VectorXd x(n_variables);
    x(0) = 1.234; x(1) = 5.678;

    Eigen::VectorXd upper_bound(n_constraints);
    Eigen::VectorXd lower_bound(n_constraints);

    for (int i = 0; i < n_constraints; ++i) {
      upper_bound(i) = OsqpEigen::INFTY;
      lower_bound(i) = 0.0;
    }

    // This declaration is pretty long. We recommend to use helper function.
    // SLSQPSolver<decltype(cost_function_gradient), decltype(inequality_constraint), decltype(inequality_constraint_gradient)> solver(n_variables, n_constraints, cost_function_gradient, inequality_constraint, inequality_constraint_gradient, lower_bound, upper_bound, x, 1e-4, 1e-3);

    auto solver = makeSLSQPSolver(n_variables, n_constraints, cost_function_gradient, inequality_constraint, inequality_constraint_gradient, lower_bound, upper_bound, x, 1e-4, 1e-3);

    solver.solve();
    std::cout << "solution" << std::endl;
    std::cout << solver.getLastX() << std::endl;
    std::cout << std::endl;

    std::cout << "optimized value" << std::endl;
    std::cout << std::sqrt(solver.getLastX()(1)) << std::endl;
    std::cout << std::endl;

    std::cout << "log" << std::endl;
    for (const auto& x : solver.getXLog()) {
      std::cout << x << std::endl;
      std::cout << std::endl;
    }

  }
};

int main()
{
  SampleSLSQP solver;
  solver.solve();
  return 0;
}
