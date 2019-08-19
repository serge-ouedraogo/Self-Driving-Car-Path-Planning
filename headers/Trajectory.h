#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include <iostream>
#include "helpers.h"
#include "Vehicle.h"
#include "JMT.h"

class Trajectory
{
  public:
    Trajectory(Vehicle& egoCar, const MotionPlanner motiontype);
    State NextState_s;
    State NextState_d;
    JMT gen_jmt_s() const;
    JMT gen_jmt_d() const;
  
  private:
    std::vector <JMT>gen_jmt;
};    
#endif