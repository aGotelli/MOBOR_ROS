#ifndef ROBOT_2_0_H
#define ROBOT_2_0_H

#include <mobile_robots/robot_base.h>

//This is a very basic (2, 0) ideal robot

struct Robot_2_0 : public Robot
{

  Robot_2_0(int _argc, char** _argv, const double& _L, const double& _r, const double& _b, const double& _wmax, const double& _x=0, const double& _y=0, const double _theta=0, const int& _frameRate=100):
            Robot(_argc, _argv, _L, _r, _b, _wmax, _x, _y, _theta, _frameRate) {}

  void Odometry() override;

  void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) override;

};


void Robot_2_0::Odometry()
{
  v = (wl + wr)*r/2;
  w = r*(wl - wr)/(2*b);

  //compute the component of Velocities
  vx = v*cos(estimatedPose.theta);
  vy = v*sin(estimatedPose.theta);

  // update position
  estimatedPose.x += vx;
  estimatedPose.y += vy;
  estimatedPose.theta += w;

}


void Robot_2_0::TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist)
{

  //processing it as in mobile robot in ARPRO
  unsigned int alpha = 20;

  double v = _Twist->linear.x;
  double omega = _Twist->linear.y*alpha +_Twist->angular.z;

  wl = (v + b*omega)/r;
  wr = (v - b*omega)/r;

  double a = std::max(abs(wl)/wmax, abs(wr)/wmax) < 1 ? 1 : std::max(abs(wl)/wmax, abs(wr)/wmax);

  wl /= a;
  wr /= a;

 }







#endif    //ROBOT_2_0_H
