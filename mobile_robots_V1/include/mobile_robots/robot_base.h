#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>    //the received Twist
#include <geometry_msgs/Pose2D.h>

#include <string.h>

#include <eigen3/Eigen/Dense>       //easy vector operations


class Robot
{
public:

  Robot(int _argc, char** _argv, const double& _L, const double& _r, const double& _b,
          const double& _wmax, const double& _x=0, const double& _y=0, const double _theta=0, const int& _frameRate=0);

  virtual void Odometry() = 0;  //depends on how accurete is the robot model

  virtual void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) = 0;    //different robot may interpretate the Twist in different ways

  virtual void IsMoving();

protected:

  ros::NodeHandle robotNode;
  ros::Subscriber desiredTwist;
  ros::Publisher robotOdometry;

  //frame rate of the robot
  ros::Rate robotFrameRate;

  //actual robot position that will be published
  geometry_msgs::Pose2D estimatedPose;

  //angular position of the wheels;
  double qr, ql;

  //angular speed of the wheels and the upper bound
  double wr, wl, wmax;

  //speed of the robot in xy plane
  double vx, vy;

  //speed of the robot in its own frame
  double v, w;

  //radius and track gauge of the wheels
  const double r, b, L;

  //useful for a (1, 1) robot
  double beta;



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

  //generally every robot need to be initialize in the same way so the constructor is the same

  //to keep everithing more ordinate I can use the desired_twist for each robot obtained in the same way
  //the CallBack function is a memeber function that is defined as always, the only thing that changes is that as it is called
  //from a subsicriber that is inside of the class as well so the function refers to UDT itself with "this"
  desiredTwist = robotNode.subscribe("TwistToRobot", 1, &Robot::TwistToProcess, this);


  robotOdometry = robotNode.advertise<geometry_msgs::Pose2D>("RobotPose", 1);

  estimatedPose.x = _x;                    //  [m]
  estimatedPose.y = _y;                    //  [m]
  estimatedPose.theta = _theta;             //  [RAD]
}


void Robot::IsMoving()
{

  while (ros::ok()) {

    ros::spinOnce();

    Odometry();

		robotOdometry.publish(estimatedPose);

		robotFrameRate.sleep();

  }

}







#endif  //ROBOT_BASE
