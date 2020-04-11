#ifndef POSE_H
#define POSE_H


#include <mobile_robots/Reference.h>
#include <geometry_msgs/Pose2D.h>        //needed for receve the position messages

#include <eigen3/Eigen/Dense>         //easy vector operations



//practical structure for points
struct Pose
{
  Pose() : x(0), y(0), z(0), theta(0), vx(0), vy(0), vz(0), omega(0){ /*default constructor*/ }

  Pose(double _x, double _y, double _z=0, double _theta=0,
        double _vx=0, double _vy=0, double _vz=0, double _omega=0 ) :
        x(_x), y(_y), z(_z), theta(_theta),
        vx(_vx), vy(_vy), vz(_vz), omega(_omega)
  {}


  //allows to save the robot estimated postion in an elegant way
  Pose& operator=(const geometry_msgs::Pose2D& _robotPose);

  //allows to save the target postion in an elegant way
  Pose& operator=(const mobile_robots::Reference& _targetRef);



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

Pose& Pose::operator=(const mobile_robots::Reference& _targetRef)
{

  this->x = _targetRef.pose.x;
  this->y = _targetRef.pose.y;
  this->z = _targetRef.pose.z;

  this->vx = _targetRef.dot_pose.vx;
  this->vy = _targetRef.dot_pose.vy;

  this->ax = _targetRef.ddot_pose.ax;
  this->ay = _targetRef.ddot_pose.ay;

  return *this;
}









#endif    //POSE_H
