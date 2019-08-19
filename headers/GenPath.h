#ifndef GenPath_H_
#define GenPath_H_

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

#include "spline.h"
#include "JMT.h"
#include "helpers.h"

class GenPath
{
  public:
    GenPath(std::string map_file_, const double distance);
    std::vector<double> Frenet_to_Cartesian(const double s, const double d)const; 
    WayPoints getpath(JMT jmt_s, JMT jmt_d, const double t, const int num_waypoints) const;
    
  private:
  double distance;
  tk::spline x_spline;
  tk::spline y_spline;
  tk::spline dx_spline;
  tk::spline dy_spline;
};
#endif