The goal of this project is to safely navigate egoCar around a virtual highway within speed limit. The code for this project consists of three main classes. 
Within the class BehaviorPlanner is the method upate which takes as input data regarding vehicle egoCar and the other vehicles and returns as output on the three optional motions that egoCar would like to undertake:
 1. Turn Left (TL)
 2. Turn Right (TR)
 3. Kepp Lane (KL)
 The method get_spacing returns the distance between egoCar and the car straight ahead on its lane as well as the distance with other cars (front and behind) on the adjacent lanes. 

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
 
 
 The calculated distance is used as an input to a cost function that weighs in the decision to turn right, turn let or keep the lane. As shown in the code below, egoCar will choose the option for which the cost function returns the lowest number.  
  
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
  
  Once egoCar decides to undertake a motion (TL, TR or KL) now comes the time to design the trajectory to go from the initial state to the goal state. (A state a data strucuture consisting egoCar's location coordinates and its speed). For this part of the project a continuous trajectory was generated using the algorithm Jerk Minimization Trajectory (JMT)
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

  Finally, The resulting trajectory is converted to a driving pathway by the method get_path that takes as input the trajectory along the Frenet coordinates to yield a set of waypoints along with their coordinates in the cartesian coordinate system.
  WayPoints GenPath::getpath(JMT jmt_s, JMT jmt_d, const double t, const int num_waypoints)
{
  vector<double> wayPts_x;
  vector<double> wayPts_y;
  for(int i = 0; i < num_waypoints; ++i)
  {
    double s = jmt_s.gen_trajectory(i * t);
    double d = jmt_d.gen_trajectory(i * t);
    std::vector<double> pathXY = this->Frenet_to_Cartesian(s, d);
    wayPts_x.push_back(pathXY[0]);
    wayPts_y.push_back(pathXY[1]);
  }

  WayPoints path = {wayPts_x, wayPts_y, num_waypoints};
  return path;
}