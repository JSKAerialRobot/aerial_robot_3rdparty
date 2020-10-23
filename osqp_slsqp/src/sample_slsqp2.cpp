#include <osqp_slsqp/slsqp.h>
#include <cmath>

/**********************************/
/*
The problem is original and contains equality constraint

variable: x in R^2 (2D vector)

min sqrt(x_2)

s.t. x_2 >= 0                  -- (1)
     x_2 >= x_1^2 - x_1 + 5/4  -- (2)
     x_2 >= x_1^2              -- (3)

ans: 5/4

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
      constraint << x(1), (x(1) - (x(0) * x(0) - x(0) + (5.0 / 4.0))), x(1) - x(0) * x(0); //(1), (2), (3)
      return constraint;
    };

    auto inequality_constraint_gradient = [this](const Eigen::VectorXd& x) -> Eigen::MatrixXd
    {
      Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(this->n_constraints, this->n_variables);
      Eigen::VectorXd inequality_grad1(this->n_variables), inequality_grad2(this->n_variables), inequality_grad3(this->n_variables);
      inequality_grad1 << 0.0, 1.0;
      inequality_grad2 << -2 * x(0) + 1, 1.0;
      inequality_grad3 << -2 * x(0), 1.0;
      grad << inequality_grad1.transpose(), inequality_grad2.transpose(), inequality_grad3.transpose();
      return grad;
    };

    Eigen::VectorXd x(n_variables);
    x(0) = 4.0; x(1) = 5.0;

    Eigen::VectorXd upper_bound(n_constraints);
    Eigen::VectorXd lower_bound(n_constraints);

    upper_bound << OsqpEigen::INFTY, OsqpEigen::INFTY, 0.0;
    lower_bound << 0.0, 0.0, 0.0;

    // This declaration is pretty long. We recommend to use helper function.
    // SLSQPSolver<decltype(cost_function_gradient), decltype(inequality_constraint), decltype(inequality_constraint_gradient)> solver(n_variables, n_constraints, cost_function_gradient, inequality_constraint, inequality_constraint_gradient, lower_bound, upper_bound, x, 1e-4, 1e-3);

    auto solver = makeSLSQPSolver(n_variables, n_constraints, cost_function_gradient, inequality_constraint, inequality_constraint_gradient, lower_bound, upper_bound, x, 1e-8, 1e-8);

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
