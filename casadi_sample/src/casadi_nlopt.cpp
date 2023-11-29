#include <casadi/casadi.hpp>
#include <CasadiEigen/CasadiEigen.h>
#include <nlopt.hpp>

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

class casadiSample
{
private:
  // casadi::SX x_sym_;
  // casadi::SX y_sym_;
  // casadi::DM x_num_;
  // casadi::DM y_num_;

public:
  int n_variables_;
  int n_constraints_;

  nlopt::opt nl_solver_;

  casadi::SX x_sym_;
  casadi::SX cost_function_sym_;
  casadi::SX cost_function_grad_sym_;
  casadi::SX inequality_constraint_sym_;
  casadi::SX inequality_constraint_grad_sym_;
  casadi::Function cost_func_calc_;
  casadi::Function cost_func_grad_calc_;
  casadiSample();

  void sample();
};

double costFunction(const std::vector<double> &x, std::vector<double> &grad, void* ptr)
{
  casadiSample *sample = reinterpret_cast<casadiSample*>(ptr);
  casadi::DM x_num = casadi::DM(x);

  casadi::DM cost_func_num = sample->cost_func_calc_(x_num);
  // std::cout << cost_func_num << std::endl;
  Eigen::VectorXd cost_func_grad_num = casadiDmToEigenVector(sample->cost_func_grad_calc_(x_num).at(0));

  // std::cout << cost_func_grad_num << std::endl;
  // std::cout << std::endl;

  // std::cout << "cost_func1" << std::endl;
  // casadi::DM cost_func = cost_func_func(x_num).at(0);
  // casadi::DM cost_func_grad = cost_func_grad_func(x_num).at(0);

  // std::cout << grad.size() << std::endl;
  if(!grad.empty())
    {
      for(int i = 0; i < grad.size(); i++)
        {
          grad.at(i) = cost_func_grad_num(i);
        }
    }
  return double(cost_func_num);
}

double inequalityConstraint(const std::vector<double> &x, std::vector<double> &grad, void* ptr)
{
  casadiSample *sample = reinterpret_cast<casadiSample*>(ptr);
}

casadiSample::casadiSample()
{
  n_variables_ = 2;
  n_constraints_ = 3;

  x_sym_ = casadi::SX::sym("x", n_variables_);
  inequality_constraint_sym_ = casadi::SX::sym("ineq_sym", n_constraints_);

  // y_sym_ = casadi::SX::sym("y", n_constraints_);
}



void casadiSample::sample()
{
  nl_solver_ = nlopt::opt(nlopt::LD_SLSQP, n_variables_);
  nl_solver_.set_min_objective(costFunction, this);
  nl_solver_.add_inequality_constraint(inequalityConstraint, this, 1e-4);
  nl_solver_.set_xtol_rel(1e-4);
  nl_solver_.set_maxeval(1000);

  cost_function_sym_ = sqrt(x_sym_(1));

  inequality_constraint_sym_(0) = -x_sym_(1);
  inequality_constraint_sym_(1) = -x_sym_(1) + pow(2 * x_sym_(0) + 0, 3);
  inequality_constraint_sym_(2) = -x_sym_(1) + pow(-1 * x_sym_(0) + 1, 3);

  cost_function_grad_sym_ = jacobian(cost_function_sym_, x_sym_);
  inequality_constraint_grad_sym_ = jacobian(inequality_constraint_sym_, x_sym_);

  cost_func_calc_ = casadi::Function("cost_func_func", {x_sym_}, {cost_function_sym_});
  cost_func_grad_calc_ = casadi::Function("cost_func_grad_func", {x_sym_}, {cost_function_grad_sym_});

  std::cout << cost_function_sym_ << std::endl;
  std::cout << std::endl;
  std::cout << cost_function_grad_sym_ << std::endl;
  std::cout << std::endl;
  std::cout << inequality_constraint_sym_ << std::endl;
  std::cout << std::endl;
  std::cout << inequality_constraint_grad_sym_ << std::endl;
  std::cout << std::endl;

  std::vector<double> initial_x(2);
  initial_x.at(0) = 1.234;
  initial_x.at(1) = 5.678;

  std::cout << "hoge1" << std::endl;
  casadi::DM initial_x_casadi = casadi::DM(initial_x);
  // std::cout << "hoge2" << std::endl;
  casadi::DM cost_func_num = cost_func_calc_(initial_x_casadi);
  // std::cout << "hoge3" << std::endl;
  casadi::DM cost_func_grad_num = cost_func_grad_calc_(initial_x_casadi).at(0);
  // std::cout << "hoge4" << std::endl;

  // std::cout << initial_x_casadi << std::endl;
  // std::cout << std::endl;
  // std::cout << cost_func_num << std::endl;
  // std::cout << std::endl;
  // std::cout << cost_func_grad_num << std::endl;
  // std::cout << std::endl;

  // std::cout << cost_func_grad_num(0) << std::endl;
  // std::cout << cost_func_grad_num(1) << std::endl;

  double diff;
  nlopt::result result;

  try
    {
      result = nl_solver_.optimize(initial_x, diff);
    }
  catch(nlopt::roundoff_limited)
    {
      std::cout << "[control][nlopt] catch nlopt::roundoff_limited" << std::endl;
    }

  if(result != nlopt::SUCCESS)
    {
      std::cout << "the optimize solution does not succeed, result is " << result << std::endl;
    }


  // casadi::Function f = casadi::Function("f", {x_sym_}, {y_sym_});

  // Eigen::VectorXd x_eigen = Eigen::VectorXd(5);
  // x_eigen << M_PI / 2.0, M_PI / 2.0, M_PI / 4.0, M_PI / 4.0, M_PI / 4.0;
  // x_num_ = eigenVectorToCasadiDm(x_eigen);
  // y_num_ = f(x_num_).at(0);
  // std::cout << "actual value of ";
  // std::cout << y_sym_;
  // std::cout << " at ";
  // std::cout << x_num_ << " is ";
  // std::cout << y_num_ << std::endl;
  // std::cout << std::endl;
  // std::cout << std::endl;

  // auto j_sym = jacobian(y_sym_, x_sym_);
  // casadi::Function j = casadi::Function("j", {x_sym_}, {j_sym});
  // auto j_num = j(x_num_);
  // std::cout << "actual value of ";
  // std::cout << j_sym << std::endl;
  // std::cout << " at " << std::endl;
  // std::cout << x_num_ << " is " << std::endl;;
  // std::cout << j_num << std::endl;
  // std::cout << std::endl;
}



int main()
{
  casadiSample casadi_sample;
  casadi_sample.sample();
  return 0;
}
