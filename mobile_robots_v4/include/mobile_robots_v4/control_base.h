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
#include <mobile_robots_v4/pose.h>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <mobile_robots_v4/simulation.h>



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


  virtual void UpdatePosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _pose);


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
  mutable Pose targetPose, robotPose;

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

void Controller::UpdatePosition(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& _pose)
{
  targetPose.x = _pose->pose.pose.position.x;
  targetPose.y = _pose->pose.pose.position.y;

  sim::Quaternion quat;
  quat.x = _pose->pose.pose.orientation.x;
  quat.y = _pose->pose.pose.orientation.y;
  quat.z = _pose->pose.pose.orientation.z;
  quat.w = _pose->pose.pose.orientation.w;

  sim::EulerAngles euler = ToEulerAngles(quat) ;

  targetPose.theta = euler.yaw ;

  targetPose.vx = cos(targetPose.theta) ;
  targetPose.vy = sin(targetPose.theta) ;

}



#endif    //CONTROL_BASE_H
