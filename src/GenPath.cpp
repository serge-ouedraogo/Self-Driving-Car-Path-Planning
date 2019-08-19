#include "GenPath.h"
using namespace std;
GenPath::GenPath(std::string map_file_, const double distance)
{
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
  double x, x1;
  double y, y1;
  float s;
  float dx, dx1;
  float dy, dy1;
  
  bool FirstLap = true;
  ifstream in_map_(map_file_.c_str(), ifstream::in);
  string line;
  
  while(getline(in_map_, line))
  {
    istringstream iss(line);
    
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(dx);
    map_waypoints_dy.push_back(dy);
    
   if (FirstLap) 
   {
     x1 = x;
     y1 = y;
     dx1 = dx;
     dy1 = dy;
     FirstLap = false;
   }
  }
 
  if(in_map_.is_open())
  {
    in_map_.close();
  }
  map_waypoints_x.push_back(x1);
  map_waypoints_y.push_back(y1);
  map_waypoints_s.push_back(distance);
  map_waypoints_dx.push_back(dx1);
  map_waypoints_dy.push_back(dy1);
  
   //cout << "number of points:" << map_waypoints_s.size() << endl;
  
  this->x_spline.set_points(map_waypoints_s, map_waypoints_x);
  this->y_spline.set_points(map_waypoints_s, map_waypoints_y);
  this->dx_spline.set_points(map_waypoints_s, map_waypoints_dx);
  this->dy_spline.set_points(map_waypoints_s, map_waypoints_dy);
  this->distance = distance;
}


vector<double> GenPath::Frenet_to_Cartesian(const double s, const double d) const  
{  
  
  double modulo_s = fmod(s, this->distance);
  //double snew;
 
  
  std::cout << " s = " << s << "  " << " distance = " << distance << "  " << " modulo = " << modulo_s << std::endl;
  
  const double seg_x = this->x_spline(modulo_s);
  const double seg_y = this->y_spline(modulo_s);  
  const double dx = this->dx_spline(modulo_s);
  const double dy = this->dy_spline(modulo_s);
   
  const double x = seg_x + dx * d;
  const double y = seg_y + dy * d;
  return {x, y};
}
  

WayPoints GenPath::getpath(JMT jmt_s, JMT jmt_d, const double t, const int num_waypoints)const 
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