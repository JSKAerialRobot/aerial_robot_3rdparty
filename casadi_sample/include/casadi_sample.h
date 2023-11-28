#pragma once
#include <casadi/casadi.hpp>

class casadiSample
{
private:
  casadi::SX A_;
  casadi::SX x_;
  casadi::SX y_;
  casadi::SX grad_y_;

public:
  casadiSample();

  void hoge();

};
