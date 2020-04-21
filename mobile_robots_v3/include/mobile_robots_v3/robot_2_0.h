/**
 * \file  (2,0) robot header file
 * \brief this file contains the model implementation of a (2,0) robot
 * \author  Andrea Gotelli
 * \version 0.1
 * \date  11 April 2020
 *
 * \param[in] none
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
 *
 *
 */



//CPP
#ifndef ROBOT_2_0_H
#define ROBOT_2_0_H
#include <mobile_robots_v3/robot_base.h>


//ROS
#include <mobile_robots_v3/Robot_2_0Config.h>


typedef mobile_robots_v3::Robot_2_0Config RobotConfig;

class Robot_2_0 : public Robot
{

public:
  Robot_2_0(const double& _r, const double& _b, const double& _wmax, const double& _x=0, const double& _y=0, const double _theta=0, const int& _frameRate=1000):
            ql(0),                                  //  [RAD]
            qr(0),                                  //  [RAD]
            wr(0),                                  //  [RAD/s]
            wl(0),                                  //  [RAD/s]
            r(_r),                                  //  [m]
            b(_b),                                  //  [m]
            Robot(_wmax, _x, _y, _theta, _frameRate) {

              dynamic_reconfigure::Server<RobotConfig>::CallbackType f = [&] {

                boost::function< void (RobotConfig &, int)> f2 = boost::bind(&Robot_2_0::SetParameters, this, _1, _2);

                return f2; } ();

                server.setCallback(f);

            }

  void Odometry() const override;

  void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) const override;

  inline void SetParameters(RobotConfig& config, uint32_t level) {wmax = config.wmax; };

  dynamic_reconfigure::Server<RobotConfig> server;

private:

  //angular position of the wheels;
  mutable double qr, ql;

  //angular speed of the wheels
  mutable double wr, wl;

  //radius and track gauge of the wheels
  const double r, b;

};


void Robot_2_0::Odometry() const
{
  //from the wheels velocities compute the robot velocities
  v = (wl + wr)*r/2;         //  [m/s]
  w = r*(wl - wr)/(2*b);     //  [RAD/s]

  //compute the component of Velocities
  vx = v*cos(estimatedPose.theta);  //  [m/s]
  vy = v*sin(estimatedPose.theta);  //  [m/s]

  //compute the displacement
  currentTime = ros::Time::now();
  timeElapsed = currentTime - prevTime;
  prevTime = currentTime;

  qr += wr*timeElapsed.toSec()*180/M_PI;
  ql -= wl*timeElapsed.toSec()*180/M_PI;

  delta_x = vx*timeElapsed.toSec();    //  [m]
  delta_y = vy*timeElapsed.toSec();    //  [m]
  delta_theta = w*timeElapsed.toSec(); //  [RAD]

  // update position
  estimatedPose.x += delta_x;
  estimatedPose.y += delta_y;
  estimatedPose.theta += delta_theta;

}


void Robot_2_0::TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) const
{

  double v = _Twist->linear.x;      //  [m/s]
  double omega = _Twist->angular.z; //  [RAD/s]

  wl = (v + b*omega)/r;   //  [RAD/s]
  wr = (v - b*omega)/r;   //  [RAD/s]

  double a = std::max(abs(wl)/wmax, abs(wr)/wmax) < 1 ? 1 : std::max(abs(wl)/wmax, abs(wr)/wmax);

  wl /= a;   //  [RAD/s]
  wr /= a;   //  [RAD/s]

}





#endif    //ROBOT_2_0_H
