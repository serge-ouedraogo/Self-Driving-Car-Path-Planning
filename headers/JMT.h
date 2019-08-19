#ifndef JMT_H_
#define JMT_H_

#include <iostream>
#include "../src/Eigen-3.3/Eigen/Core"
#include "../src/Eigen-3.3/Eigen/QR"
#include "../src/Eigen-3.3/Eigen/Dense"
#include "helpers.h"

class JMT
{
  public:
    Eigen::VectorXd state_s_param;
    JMT(const State& initial, const State& final, const double T);
    double gen_trajectory(const double t) const;
  
};
#endif