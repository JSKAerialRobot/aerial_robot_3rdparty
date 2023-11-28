#include <casadi_sample.h>

casadiSample::casadiSample()
{
  A_ = casadi::SX(2, 5);
  A_(0, 0) = 1.0;
  A_(1, 1) = 1.0;
  A_(1, 2) = 1.0;
  x_ = casadi::SX::sym("x", 5);
}

void casadiSample::hoge()
{
  std::cout << x_ << std::endl;
  std::cout << std::endl;

  std::cout << A_ << std::endl;
  std::cout << std::endl;

  std::cout << A_.size1() << " " << A_.size2() << std::endl;

  y_ = mtimes(A_, x_);
  std::cout << y_ << std::endl;
  std::cout << std::endl;

  auto j = jacobian(y_, x_);
  std::cout << j << std::endl;
  std::cout << std::endl;
}


int main()
{
  casadiSample casadi_sample;
  casadi_sample.hoge();
  return 0;
}
