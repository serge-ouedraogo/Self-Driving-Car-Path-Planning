#include "JMT.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

JMT::JMT(const State& initial, const State& final, const double T)
{
  MatrixXd A = MatrixXd(3,3);
  VectorXd b = VectorXd(3);
  VectorXd x =  VectorXd(3);
  this->state_s_param = VectorXd(6);
  
  A << T*T*T, T*T*T*T, T*T*T*T*T, 
       3*T*T, 4*T*T*T, 5*T*T*T*T, 
       6*T, 12*T*T, 20*T*T*T ; 
  
  b << final.position - (initial.position + initial.speed*T + 0.5*initial.acceleration*T*T),
       final.speed - (initial.speed + initial.acceleration*T),
       final.acceleration - initial.acceleration ;
  
  x = A.inverse() * b;
  this->state_s_param << initial.position, 
                         initial.speed, 
                         initial.acceleration, 
                         x[0], 
                         x[1], 
                         x[2]; 
}
  
double JMT::gen_trajectory(const double t) const
{
  VectorXd T = VectorXd(6);
  T <<1.0, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
  
  return T.transpose() * this->state_s_param;
  
}