#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <iostream>
#include <vector>
#include <math.h>
#include "helpers.h"

class Vehicle
{
public:
  Vehicle( const int i);
  
  int vehicle_id;
  double s;
  double d;
  double speed;
  double front_space;
  double front_s;
  double front_speed;
  double back_space;
  
  Lane lane;
  Lane left_lane;
  Lane right_lane;
  
  State current_state_s;
  State current_state_d;
  
  void update_position(const double s, const double d);
  void update_speed(const double speed);
  void update_state(const State& current_s, const State& current_d);
  
  void neighboring_lane();
  
  Lane detectLane();
  Lane detectLane(const double d);
  double get_d_from_lane(const Lane lane);
  double get_d_from_lane();
  double get_target_d(const MotionPlanner motiontype);
};
#endif