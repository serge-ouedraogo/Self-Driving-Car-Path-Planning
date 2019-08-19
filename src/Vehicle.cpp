#include "Vehicle.h"


Vehicle::Vehicle(const int id)
{
  this->vehicle_id = id;
  this->lane = Lane::Unspecified;
}

void Vehicle::update_position( const double s, const double d)
{
  this->s = s;
  this->d = d;
  this->lane = this->detectLane(this->d);
}

void Vehicle::update_speed( const double speed)
{
  this->speed = speed;
}
void Vehicle::update_state(const State& state_s,  const State& state_d)
{
  this->current_state_s = state_s;
  this->current_state_d = state_d;
  this->current_state_s.position = fmod(this->current_state_s.position, Track_Length);
}
  
void Vehicle::neighboring_lane()
{
  if(this->lane ==Lane::Left)
  {
    this->right_lane = Lane::Mid;
    this->left_lane = Lane::None;
  }
  
  else if(this->lane ==Lane::Mid)
  {
    this->right_lane = Lane::Right;
    this->left_lane = Lane::Left;
  }
  else if(this->lane ==Lane::Right)
  {
    this->right_lane = Lane::None;
    this->left_lane = Lane::Mid;
  }
  else
  {
    this->lane = Lane::Unspecified;
    this->right_lane = Lane::Unspecified;
    this->left_lane = Lane::Unspecified;
  }
}

Lane Vehicle::detectLane(const double d)
{
  Lane lane = Lane::None;
  if(d > 0.0 && d < 4.0)
  {
    lane = Lane::Left;
  }
  if(d >4.0 && d < 8.0)
  {
    lane = Lane::Mid;
  }
  
   if(d >8.0 && d < 12.0)
  {
    lane = Lane::Right;
  }
  return lane;
}

Lane Vehicle::detectLane()
{
  return this->detectLane(this->d);
}

double Vehicle::get_d_from_lane(const Lane lane)
{
  double d =Mid_d;
  if(lane == Lane::Left)
    d = Left_d;
  if(lane == Lane::Right)
    d= Right_d;
  if(lane == Lane::Mid)
    d = Mid_d;
  return d;
}

double Vehicle::get_d_from_lane()
{
  return this->get_d_from_lane(this->lane);
}
  
double Vehicle::get_target_d(const MotionPlanner motiontype)
{
  if(motiontype == MotionPlanner::KeepLane)
  {
    return (this->get_d_from_lane(this->lane));
  }
  
  else if(motiontype == MotionPlanner::TurnLeft)
  {
    return (this->get_d_from_lane(this->left_lane));
  }
  
  else if(motiontype == MotionPlanner::TurnRight)
  {
    return (this->get_d_from_lane(this->right_lane));
  }
  return this->get_d_from_lane(this->lane);
  //return 0;
}
