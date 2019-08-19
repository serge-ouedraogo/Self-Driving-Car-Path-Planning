#ifndef BEHAVIORPLANNER_H_
#define BEHAVIORPLANNER_H_

#include <vector>
#include <iostream>

#include "helpers.h"
#include "Vehicle.h"


class BehaviorPlanner
{
  public:
    BehaviorPlanner();
    MotionPlanner update(Vehicle& egoCar, std::vector <Vehicle>& otherCars);
    
    double get_spacing(const Vehicle& egoCar, const std::vector <Vehicle>& otherCars, const Lane lane,  const double otherCars_pos);
  private:
    double cost_fct(const double spacing) const;
    double cost_fct(const double front_spacing, const double back_spacing,  const Lane lane) const;
    
    double front_car_speed;
    double front_car_s;
};
#endif