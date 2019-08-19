#ifndef HELPERS_H_
#define HELPERS_H_

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.

const double Left_d = 2.25;
const double Mid_d = 6.0;
const double Right_d = 9.75;

const double front_spacing_threshold = 20.0;
const double back_spacing_threshold = 15.0;

const double front_buffer = 30;
const double reduced_speed = 3.6;
const double speed_buffer = 8.0;

const double otherCars_in_front = 1.0;
const double otherCars_in_back =- 1.0;
const double Track_Length = 6945.554;

const double speed_limit = 20.75; 
const double minimum_speed = 11.0;

const double delta_time = 2.0;
const double time_increment = 0.02;
const int num_waypoints = int(delta_time / time_increment);

const double keep_lane_weight = 0.7;
const double change_lane_weight = 1.5;
const double Front_space_cost_factor = 1.2;
const double Back_space_cost_factor = 0.5;
const int MIN_WAYPOINTS = 15;
enum class Lane
{
  Mid,
  Left,
  Right,
  None,
  Unspecified
};

enum class MotionPlanner
{
  KeepLane,
  TurnLeft,
  TurnRight,
};

struct State
{
  double position;
  double speed;
  double acceleration;
};

struct WayPoints {
  std::vector<double> wayPts_x;
  std::vector<double> wayPts_y;
  int num_waypoints;
};

#endif  // HELPERS_H