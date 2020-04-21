/**
 * \file  pose header file
 * \brief this file contains the position class used in all the node of this project
 * \author  Andrea Gotelli
 * \version 0.1
 * \date  10 April 2020
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
 *        This header file contains all the function and operator ovverrides that
          make the code in the node easier and simple. so besically all the complexities
          are moved here. 
 *
 */






//CPP
#ifndef POSE_H
#define POSE_H


#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>        //needed for receve the position messages

#include <eigen3/Eigen/Dense>         //easy vector operations



//practical structure for points
struct Pose
{
  Pose() : x(0), y(0), z(0), theta(0), vx(0), vy(0), vz(0), omega(0),
            ax(0), ay(0), az(0), omega_dot(0)   { /*default constructor*/ }

  Pose(double _x, double _y, double _z=0, double _theta=0,
        double _vx=0, double _vy=0, double _vz=0, double _omega=0,
          double _ax=0, double _ay=0, double _az=0, double _omega_dot=0) :
        x(_x), y(_y), z(_z), theta(_theta),
        vx(_vx), vy(_vy), vz(_vz), omega(_omega),
        ax(_vx), ay(_vy), az(_vz), omega_dot(_omega_dot)
  {}


  //allows to save the robot estimated postion in an elegant way
  Pose& operator=(const geometry_msgs::Pose2D& _robotPose);

  //allows to save the target postion in an elegant way
  Pose& operator=(const nav_msgs::Odometry& _targetRef);



  inline const Eigen::VectorXd xy() const {Eigen::VectorXd _xy(2);
                                          _xy(0) = x;
                                          _xy(1) = y;
                                          return _xy;  };

  inline const Eigen::VectorXd xy_dot() const {Eigen::VectorXd _xy_dot(2);
                                              _xy_dot(0) = vx;
                                              _xy_dot(1) = vy;
                                              return _xy_dot;  };


  double x, y, z, theta;
  double vx, vy, vz, omega;
  double ax, ay, az, omega_dot;

} ;


Pose& Pose::operator=(const geometry_msgs::Pose2D& _robotPose)
{

  this->x = _robotPose.x;
  this->y = _robotPose.y;
  this->theta = _robotPose.theta;

  return *this;
}

Pose& Pose::operator=(const nav_msgs::Odometry& _targetOdometry)
{

  this->x = _targetOdometry.pose.pose.position.x;
  this->y = _targetOdometry.pose.pose.position.y;
  this->z = _targetOdometry.pose.pose.position.z;

  this->vx = _targetOdometry.twist.twist.linear.x;
  this->vy = _targetOdometry.twist.twist.linear.y;

  return *this;
}





#endif    //POSE_H
