#include <casadi/casadi.hpp>

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

int main()
{
  // variable
  casadi::MX x = casadi::MX::sym("x", 2);

  bool verbose = true;

  // cost function
  casadi::MX objective = sqrt(x(1));

  // constraints
  std::vector<casadi::MX> constraints;
  constraints.push_back(-x(1));
  constraints.push_back(-x(1) + pow((2 * x(0) + 0), 3));
  constraints.push_back(-x(1) + pow((-1 * x(0) + 1), 3));

  if(verbose) std::cout << constraints << std::endl;

  casadi::MXDict nlp = {{"x", x}, {"f", objective}, {"g", vertcat(constraints)}};

  // set print level silently
  casadi::Dict opt_dict = casadi::Dict();
  if(!verbose)
    {
      opt_dict["ipopt.print_level"] = 0;
      opt_dict["ipopt.sb"] = "yes";
      opt_dict["print_time"] = 0;
    }

  casadi::Function S = casadi::nlpsol("S", "ipopt", nlp, opt_dict);

  casadi::DM initial_x = casadi::DM({1.234, 5.678});
  casadi::DM ubg = casadi::DM({0.0, 0.0, 0.0});
  casadi::DM lbg = casadi::DM({-INFINITY, -INFINITY, -INFINITY});
  casadi::DM lbx = casadi::DM({-INFINITY, -INFINITY});
  casadi::DM ubx = casadi::DM({INFINITY, INFINITY});

  auto res = S(casadi::DMDict{{"x0", initial_x}, {"lbg", lbg}, {"ubg", ubg}, {"lbx", lbx}, {"ubx", ubx}});
  std::cout << res << std::endl;
}
