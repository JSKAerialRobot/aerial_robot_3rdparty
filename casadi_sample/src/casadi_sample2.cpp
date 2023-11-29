#include <casadi/casadi.hpp>

/**********************************/
/*
The problem is original and contains equality constraint

variable: x in R^2 (2D vector)

min 2x_1 + x_2^2

s.t. 2x_1^2 + x_2^2 = 1                  -- (1)

ans -sqrt(2) at [- sqrt(2) / 2, 0]

*/
/**********************************/

int main()
{
  // variable
  casadi::MX x = casadi::MX::sym("x", 2);

  bool verbose = true;

  // cost function
  casadi::MX objective = 2 * x(0) + x(1) * x(1);

  // constraints
  std::vector<casadi::MX> constraints;
  constraints.push_back(2 * x(0) * x(0) + x(1) * x(1) - 1);

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

  casadi::DM initial_x = casadi::DM({0.0, 0.0});
  casadi::DM lbg = casadi::DM({0.0});
  casadi::DM ubg = casadi::DM({0.0});
  casadi::DM lbx = casadi::DM({-INFINITY, -INFINITY});
  casadi::DM ubx = casadi::DM({INFINITY, INFINITY});

  auto res = S(casadi::DMDict{{"x0", initial_x}, {"lbg", lbg}, {"ubg", ubg}, {"lbx", lbx}, {"ubx", ubx}});
  std::cout << res << std::endl;
}
