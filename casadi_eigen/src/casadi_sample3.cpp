#include <casadi/casadi.hpp>
#include <CasadiEigen/CasadiEigen.h>

/**********************************/
/*
The problem is from OSQP tutorial

variable: x in R^2 (2D vector)

min 1/2 x^T A x + p^T x

s.t. A = [4, 1
          1, 2]

     p = [1, 1]^T

     lb <= Gx <= ub         -----(1)
     lb = [1, 0, 0]^T
     G = [1, 1
          1, 0
          0, 1]
     ub = [1, 0.7, 0.7]^T

*/
/**********************************/


int main()
{
  int n_variables = 2;
  int n_constraints = 3;

  bool verbose = true;

  casadi::MX x = casadi::MX::sym("x", 2);

  Eigen::MatrixXd A;
  A.resize(n_variables, n_variables);
  A <<
    4.0, 1.0,
    1.0, 2.0;

  Eigen::VectorXd p;
  p.resize(n_variables);
  p <<
    1.0, 1.0;

  Eigen::VectorXd lbg;
  lbg.resize(n_constraints);
  lbg <<
    1.0, 0.0, 0.0;

  Eigen::VectorXd ubg;
  ubg.resize(n_constraints);
  ubg <<
    1.0, 0.7, 0.7;

  Eigen::MatrixXd G;
  G.resize(n_constraints, n_variables);
  G <<
    1.0, 1.0,
    1.0, 0.0,
    0.0, 1.0;

  casadi::MX objective = 1.0 / 2.0 * dot(x, mtimes(eigenMatrixToCasadiDM(A), x)) + dot(eigenVectorToCasadiDm(p), x);
  casadi::MX constraints = mtimes(eigenMatrixToCasadiDM(G), x);

  casadi::MXDict nlp = {{"x", x}, {"f", objective}, {"g", constraints}};
  casadi::Dict opt_dict = casadi::Dict();
  if(!verbose)
    {
      opt_dict["ipopt.print_level"] = 0;
      opt_dict["ipopt.sb"] = "yes";
      opt_dict["print_time"] = 0;
    }
  casadi::Function S = casadi::nlpsol("S", "ipopt", nlp, opt_dict);

  casadi::DM initial_x = casadi::DM({1.234, 5.678});
  auto res = S(casadi::DMDict{{"x0", initial_x}, {"lbg", eigenVectorToCasadiDm(lbg)}, {"ubg", eigenVectorToCasadiDm(ubg)}});

  std::cout << res << std::endl;
}
