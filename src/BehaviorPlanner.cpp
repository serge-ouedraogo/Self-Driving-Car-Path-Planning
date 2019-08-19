#include "BehaviorPlanner.h"

using namespace std;

// Constructor
BehaviorPlanner::BehaviorPlanner() {}

//define the update Method
MotionPlanner BehaviorPlanner:: update(Vehicle& egoCar, std::vector<Vehicle>& otherCars)
{
  egoCar.front_speed = this->front_car_speed;
  egoCar.front_s = this->front_car_s;
  egoCar.front_space = this->get_spacing(egoCar, otherCars, egoCar.lane, otherCars_in_front);
  const double keeplane_cost = this->cost_fct(egoCar.front_space);
  
  const double front_space_left = this->get_spacing(egoCar, otherCars, egoCar.left_lane, otherCars_in_front);
  const double back_space_left = this->get_spacing(egoCar, otherCars, egoCar.left_lane, otherCars_in_back);
  const double turnleft_cost = this->cost_fct(front_space_left, back_space_left, egoCar.left_lane);
  
  const double front_space_right = this->get_spacing(egoCar, otherCars, egoCar.right_lane, otherCars_in_front);
  const double back_space_right = this->get_spacing(egoCar, otherCars, egoCar.right_lane, otherCars_in_back);
  const double turnright_cost = this->cost_fct(front_space_right, back_space_right, egoCar.right_lane);
  
  if((keeplane_cost > turnleft_cost) && (turnleft_cost < turnright_cost))
  {
   /*  
    std::cout << "****" << " Keep Lane Cost:" << keeplane_cost << "\n" 
              << "****" << " Turn Left Cost " << turnleft_cost << "\n"
              << "****" << " Turn Right Cost" << turnright_cost << "\n"
              << "****" << "Motion: TurnLeft " << "\n" 
              << "****" << " front space right:" << front_space_right << "\n" 
              << "****" << " front space left: " << front_space_left << "\n"  
              << "****" << " back space right:" << back_space_right << "\n" 
              << "****" << " back space left: " << back_space_left << "\n" 
              << "************************************************" << std::endl;
   */
    return MotionPlanner::TurnLeft;
  }
  
  if((keeplane_cost > turnright_cost) && (turnright_cost < turnleft_cost))
  {
    /*
    std::cout << "****" << " Keep Lane Cost:" << keeplane_cost << "\n" 
              << "****" << " Turn Left Cost " << turnleft_cost << "\n"
              << "****" << " Turn Right Cost" << turnright_cost << "\n"
              << "****" << "Motion: TurnRight" << "\n"
              << "****" << " front space right:" << front_space_right << "\n" 
              << "****" << " front space left: " << front_space_left << "\n"  
              << "****" << " back space right:" << back_space_right << "\n" 
              << "****" << " back space left: " << back_space_left << "\n"
              << "************************************************" << std::endl;
    */
    return MotionPlanner::TurnRight;
  }
  /*
  std::cout << "****" << " Keep Lane Cost:" << keeplane_cost << "\n" 
              << "****" << " Turn Left Cost " << turnleft_cost << "\n"
              << "****" << " Turn Right Cost" << turnright_cost << "\n"
              << "****" << "Motion: Keep Lane" << "\n"
              << "****" << " front space right:" << front_space_right << "\n" 
              << "****" << " front space left: " << front_space_left << "\n"  
              << "****" << " back space right:" << back_space_right << "\n" 
              << "****" << " back space left: " << back_space_left << "\n"
              << "************************************************" << std::endl;
  */
  return MotionPlanner::KeepLane;
    
}

// Define the get_spacing Method
double BehaviorPlanner::get_spacing(const Vehicle& egoCar, const vector <Vehicle>& otherCars, const Lane lane, const double otherCars_pos)
{
  if( lane == Lane::None ||lane==Lane::Unspecified)
  {
    return 1E-6;
  }
  double min_space = 9999.0;
  for(auto &othercar:otherCars)
  {
    double space = (othercar.s - egoCar.s) * otherCars_pos;
    if((space > 0) && (space < min_space) && (othercar.lane == lane))
    {
      min_space = space;
    this->front_car_speed = othercar.speed;
    this->front_car_s = othercar.s;
    }
  }
  return min_space;
}
  // Define the cost_fct Method associated to the keeping lane option
double BehaviorPlanner::cost_fct(const double space) const 
{
  if(space < front_spacing_threshold)
    return 1E6;
  double cost = Front_space_cost_factor/space; 
  return cost;
}
    
// Define the cost_fct Method associated to the turning options
double BehaviorPlanner::cost_fct(const double front_space, const double back_space, const Lane lane) const
{
  if((front_space < front_spacing_threshold) || (back_space < back_spacing_threshold))
    return 1E6;
  double cost = Front_space_cost_factor/front_space + Back_space_cost_factor/back_space;
  if(lane == Lane::Mid)
  {
    cost = cost * keep_lane_weight;
  }
  else
  {
    cost = cost * change_lane_weight;
  }
  return cost;
}