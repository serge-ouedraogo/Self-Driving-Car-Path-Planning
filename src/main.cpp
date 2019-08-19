#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include<cmath>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "GenPath.h"
#include "Vehicle.h"
#include "JMT.h"
#include "BehaviorPlanner.h"
#include "Trajectory.h"

using nlohmann::json;

using std::string;
using std::vector;
using namespace std;

string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// for convenience
double start_s;
double start_d;
WayPoints start_vehicle(Vehicle& egoCar, GenPath& genpath)
{
  int n = 200; 
  double t = n * time_increment;
  
  start_s = egoCar.s;
  start_d = egoCar.d;
  double start_speed = egoCar.speed;
  
  double end_s = egoCar.s +40;
  //double end_s = egoCar.s + 50;
  double end_d = egoCar.get_d_from_lane();
  double end_speed = 20.0;
  
  std:: cout << "end_s: " << end_s << std::endl;
  std:: cout << "end_d: " << end_d << std::endl; 
  std:: cout << "n: " << n << std::endl;
 
  
  State startState_s = {start_s, start_speed, 0.0};
  State startState_d = {start_d, 0.0, 0.0};
  
  State endState_s = {end_s, end_speed, 0.0}; 
  State endState_d = {end_d, 0.0, 0.0}; 
 
  JMT jmt_s(startState_s, endState_s, t);
  JMT jmt_d(startState_d, endState_d, t);
 
  egoCar.update_state(endState_s, endState_d);
 
  return genpath.getpath(jmt_s, jmt_d, time_increment, n);
}

int main() 
{
  uWS::Hub h;
  bool start = true;
  GenPath genpath("../data/highway_map.csv", Track_Length);
    
  h.onMessage([&genpath, &start] 
  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) 
  {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') 
    {

      auto s = hasData(data);

      if (s != "") 
      {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") 
        {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          
          Vehicle egoCar(1);
          /*
          if(car_s < 10)
          {
            car_s += delta_time*(previous_path_x.size());
          }
          */
          if (previous_path_x.size() > 0) 
          {
            car_s = end_path_s;
          }
          egoCar.update_position(car_s, car_d);
          std::cout << "car_s: " << car_s << " car_d: " << car_d << std::endl;
          
          egoCar.update_speed(car_speed);
          egoCar.neighboring_lane();
          
          int n =previous_path_x.size(); 
         
          WayPoints waypts = {previous_path_x, previous_path_y, n};
          //std::cout << " n before start: " << n << std::endl;
          if(start)
          { 
            //std::cout << " n during start: " << n << std::endl;
            waypts = start_vehicle(egoCar, genpath);
            start = false;
          }
          else if( n < MIN_WAYPOINTS)
          {
            std::cout << " n afert start: " << n << std::endl;
            std::cout << "n: " << n << std::endl; 
            vector<Vehicle> OtherCars;
            for(int i =0; i < sensor_fusion.size(); i++)
            {
              int id = sensor_fusion[i][0];
              double s = sensor_fusion[i][5];
              double d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              Vehicle othercar(id);
              //std::cout << "Vehicle ID :" << id << std::endl;
              //double other_car_speed = sqrt((vx * vx) + (vy * vy));
              //s+= n*sqrt(vx*vx + vy*vy)* delta_time;
              
              othercar.update_position(s, d);
              othercar.update_speed(sqrt(vx*vx + vy*vy));
              //othercar.update_speed(other_car_speed);
              OtherCars.push_back(othercar);
            }
            BehaviorPlanner behavior;
            MotionPlanner motiontype = behavior.update(egoCar, OtherCars);
            
            //Trajectory trajectory;
            Trajectory trajectory(egoCar, motiontype);
            egoCar.update_state(trajectory.NextState_s, trajectory.NextState_d);
            //std::cout << "trajectory.NextState_d = " << trajectory.NextState_d <<std::endl; 
            WayPoints NextWayPoints = genpath.getpath(trajectory.gen_jmt_s(), trajectory.gen_jmt_d(), time_increment, num_waypoints);
          
            NextWayPoints.num_waypoints = num_waypoints;
           
            waypts.wayPts_x.insert(waypts.wayPts_x.end(),NextWayPoints.wayPts_x.begin(), NextWayPoints.wayPts_x.end());
            waypts.wayPts_y.insert(waypts.wayPts_y.end(),NextWayPoints.wayPts_y.begin(), NextWayPoints.wayPts_y.end());
            
            waypts.num_waypoints = waypts.wayPts_x.size();
            std:: cout << "WAYPOINTS_X_SIZE = " << waypts.num_waypoints << std::endl;
          }
          json msgJson;
          msgJson["next_x"] = waypts.wayPts_x;
          msgJson["next_y"] = waypts.wayPts_y;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } 
      else 
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) 
                         {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
