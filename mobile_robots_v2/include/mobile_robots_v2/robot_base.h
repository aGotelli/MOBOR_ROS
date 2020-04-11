/**
 * \file  base robot header file
 * \brief this file is the base for every robot implementation
 * \author  Andrea Gotelli
 * \version 0.1
 * \date  11 April 2020
 *
 * \param[in] none
 *
 * Subscribes to: <BR>
 *    ° "TwistToRobot"
 *
 * Publishes to: <BR>
 *    ° "RobotPose"
 *    ° "RobotToRviz"
 *
 * Description
 *        This file contains the general feature for a robot, in a way that for
          simultate any other robot, is suffiecient to define Odometry() and the
          Callback function TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist);
          In fact evry robot has it's own odometry and processes the twist in a different way.
 *
 */






//CPP
#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H
#include <mobile_robots_v2/simulation.h>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>    //the received Twist
#include <geometry_msgs/Pose2D.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>




class Robot
{
public:

  Robot(int _argc, char** _argv, const double& _L, const double& _r, const double& _b,
          const double& _wmax, const double& _x=0, const double& _y=0, const double _theta=0, const int& _frameRate=0);

  virtual void Odometry() = 0;  //depends on how accurete is the robot model

  virtual void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) = 0;    //different robot may interpretate the Twist in different ways

  virtual void IsMoving();

protected:
  //node tools
  ros::NodeHandle robotNode;
  ros::Subscriber desiredTwist;
  ros::Publisher robotOdometry;
  ros::Publisher robotToRviz;

  //frame rate of the robot
  ros::Rate robotFrameRate;

  //actual robot position that will be published
  geometry_msgs::Pose2D estimatedPose;

  //angular position of the wheels;
  double qr, ql;

  //angular speed of the wheels and the upper bound
  double wr, wl;
  mutable double wmax;

  //speed of the robot in xy plane
  double vx, vy;

  //speed of the robot in its own frame
  double v, w;

  //radius and track gauge of the wheels
  const double r, b, L;

  //useful for a (1, 1) robot
  double beta;

  //the marker of the robot
  visualization_msgs::Marker robotMarker;

};



Robot::Robot(int _argc, char** _argv, const double& _L, const double& _r, const double& _b,
              const double& _wmax, const double& _x, const double& _y, const double _theta, const int& _frameRate):
robotFrameRate(ros::Rate(_frameRate)),  //  [Hz]
beta(0),                                //  [RAD]
ql(0),                                  //  [RAD]
qr(0),                                  //  [RAD]
wr(0),                                  //  [RAD/s]
wl(0),                                  //  [RAD/s]
wmax(_wmax),                            //  [RAD/s]
L(_L),                                  //  [m]
r(_r),                                  //  [m]
b(_b)                                   //  [m]
{
  ROS_INFO_STREAM("Robot (2, 0) online...");

  desiredTwist = robotNode.subscribe("TwistToRobot", 1, &Robot::TwistToProcess, this);

  robotOdometry = robotNode.advertise<geometry_msgs::Pose2D>("RobotPose", 1);

  estimatedPose.x = _x;                    //  [m]
  estimatedPose.y = _y;                    //  [m]
  estimatedPose.theta = _theta;             //  [RAD]

  robotToRviz = robotNode.advertise<visualization_msgs::Marker>("RobotToRviz", 1);

  robotMarker = sim::RobotMarkerInit();
}


void Robot::IsMoving()
{

  while (ros::ok()) {

    ros::spinOnce();

    Odometry();

		robotOdometry.publish(estimatedPose);

    sim::SetCoordinates(Pose(estimatedPose.x, estimatedPose.y, 0, estimatedPose.theta), robotMarker);

    robotToRviz.publish(robotMarker);

		robotFrameRate.sleep();

  }

}







#endif  //ROBOT_BASE
