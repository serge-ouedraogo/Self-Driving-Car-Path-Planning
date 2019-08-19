#include "Trajectory.h"

Trajectory::Trajectory(Vehicle& egoCar, const MotionPlanner motiontype)
{
  double next_s = egoCar.current_state_s.position + (egoCar.current_state_s.speed)*delta_time;
  double next_speed = egoCar.current_state_s.speed;
  bool safe =  (egoCar.front_speed > speed_limit) || (egoCar.front_space > front_buffer);
    
  if(motiontype == MotionPlanner::KeepLane)
  {
    if(safe)
    {
      next_speed = speed_limit;
      /*
      std::cout << " egoCar speed = " << egoCar.speed << "\n"
                << " Car in front speed:" << egoCar.front_speed << "\n"
                << " Front Buffer:" << egoCar.front_space << std::endl;
      */
    }
   
    if(egoCar.front_space < front_buffer)
    {
      double new_speed = egoCar.front_speed - speed_buffer;
      if(new_speed < minimum_speed)
      {
        next_speed = minimum_speed;
        /*
        std::cout << " egoCar speed = " << egoCar.speed << "\n"
                << " Car in front speed:" << egoCar.front_speed << "\n"
                << " Front Buffer:" << egoCar.front_space << std::endl;
        */
      }
      else
      {
        next_speed = new_speed;
        /*
        std::cout << " egoCar speed = " << egoCar.speed << "\n"
                << " Car in front speed:" << egoCar.front_speed << "\n"
                << " Front Buffer:" << egoCar.front_space << std::endl;
      */
      }
    }
    next_s = egoCar.current_state_s.position + 0.5*(egoCar.current_state_s.speed + next_speed)*delta_time;
  }
  
  this->NextState_s = {next_s, next_speed, 0.0};
  this->NextState_d = {egoCar.get_target_d(motiontype),0.0, 0.0}; 
  
  //Uisng here the JMT constructor as defined in JMT.cpp (jmt_s and jmt_d are objects of JMT::JMT())
  JMT jmt_s(egoCar.current_state_s, NextState_s, delta_time);
  JMT jmt_d(egoCar.current_state_d, NextState_d, delta_time);
  this->gen_jmt.push_back(jmt_s);
  this->gen_jmt.push_back(jmt_d);
}

JMT Trajectory::gen_jmt_s() const
{
  return gen_jmt[0];
}

JMT Trajectory::gen_jmt_d() const
{
  return gen_jmt[1];
}
 



  
  
  
  
  