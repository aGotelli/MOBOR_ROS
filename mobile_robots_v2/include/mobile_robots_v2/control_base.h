/**
 * \file  base controller header file
 * \brief this file is the base for every controller implementation
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
 *        This file contains the necessary functions, like the one to store the target and robot positions.
          The file allows the creation of new controller by simply overrifding the virtual function ComputeCorrectionTwist();
 *
 */

//CPP
#ifndef CONTROL_BASE_H
#define CONTROL_BASE_H

//ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <mobile_robots_v2/pose.h>
#include <dynamic_reconfigure/server.h>



class Controller
{
public:

  Controller(const double& _d, const double& _Kp):
            d(_d),      //distance between oring of moving platform and reference point
            Kp(_Kp),    //static gain
            Kx(0),        //static gain
            Ky(0),        //static gain
            Ktheta(0)     //static gain
            {}

  Controller(const double& _Kx,const double& _Ky, const double& _Ktheta):
            Kx(_Kx),           //static gain
            Ky(_Ky),          //static gain
            Ktheta(_Ktheta),  //static gain
            d(0),      //distance between oring of moving platform and reference point
            Kp(0)     //static gain
            {}

  Controller():
            Kx(0),           //static gain
            Ky(0),          //static gain
            Ktheta(0),  //static gain
            d(0),      //distance between oring of moving platform and reference point
            Kp(0)     //static gain
            {}

  //read message from target
  virtual void UpdateTargetPosition(const nav_msgs::Odometry::ConstPtr& _targetPose);

  //read message from robot
  virtual void UpdateRobotPosition(const geometry_msgs::Pose2D::ConstPtr& _robotPose);

  //perform the control
  virtual void ComputeCorrectionTwist() = 0;


  //return the control
  inline const geometry_msgs::Twist& TwistToRobot() const {return twistToRobot;}



protected:

  //defined by me, very useful message type
  geometry_msgs::Twist twistToRobot;

  //the two element in the environment
  Pose targetPose, robotPose;

  //static feedback parameters
  mutable double d, Kp;

  //Lyapunov parameters
  mutable double Kx, Ky, Ktheta;


};



void Controller::UpdateTargetPosition(const nav_msgs::Odometry::ConstPtr& _targetPose)
{
  //need to pass the actual object not the pointer
  targetPose = (*_targetPose);
}


void Controller::UpdateRobotPosition(const geometry_msgs::Pose2D::ConstPtr& _robotPose)
{
  //ROS_INFO_STREAM("gettin some signals from a robot..." << _estimatedPose->pose.x << " " << _estimatedPose->pose.y);
  robotPose = (*_robotPose);
}



#endif    //CONTROL_BASE_H
