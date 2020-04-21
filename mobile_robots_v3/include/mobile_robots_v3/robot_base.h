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
#include <mobile_robots_v3/simulation.h>

//ROS
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>    //the received Twist
#include <geometry_msgs/Pose2D.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>



class Robot
{
public:

  Robot(const double& _wmax, const double& _x, const double& _y, const double _theta, const int& _frameRate);

  virtual void Odometry() const = 0;  //depends on how accurete is the robot model

  virtual void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) const = 0;    //different robot may interpretate the Twist in different ways

  virtual void IsMoving() const;

  virtual void poseCallback(const geometry_msgs::Pose2DConstPtr& msg) const;

protected:
  //node tools
  mutable ros::NodeHandle robotNode;
  mutable ros::Subscriber desiredTwist, tfRobot;
  mutable ros::Publisher robotOdometry;
  mutable ros::Publisher robotToRviz;

  //frame rate of the robot
  mutable ros::Rate robotFrameRate;

  //actual robot position that will be published
  mutable geometry_msgs::Pose2D estimatedPose;

  //upper bound for the wheel speed
  mutable double wmax;

  //speed of the robot in xy plane
  mutable double vx, vy;

  //speed of the robot in its own frame
  mutable double v, w;

  // robot displacements
  mutable double delta_x, delta_y, delta_theta;

  //the marker of the robot
  mutable visualization_msgs::Marker robotMarker;

  // Handling the time
  mutable ros::Time currentTime, prevTime;
  mutable ros::Duration timeElapsed;
};



Robot::Robot(const double& _wmax, const double& _x, const double& _y, const double _theta, const int& _frameRate):
robotFrameRate(ros::Rate(_frameRate)),  //  [Hz]
prevTime(ros::Time::now()),
wmax(_wmax)                             //  [RAD/s]
{

  ROS_INFO_STREAM(ros::this_node::getName() << "online...");

  desiredTwist = robotNode.subscribe("TwistToRobot", 1, &Robot::TwistToProcess, this);

  robotOdometry = robotNode.advertise<geometry_msgs::Pose2D>("RobotPose", 1);

  estimatedPose.x = _x;                    //  [m]
  estimatedPose.y = _y;                    //  [m]
  estimatedPose.theta = _theta;            //  [RAD]

  robotToRviz = robotNode.advertise<visualization_msgs::Marker>("RobotToRviz", 1);

  robotMarker = sim::RobotMarkerInit();

  tfRobot = robotNode.subscribe("RobotPose", 1, &Robot::poseCallback, this);
}


void Robot::IsMoving() const
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

void Robot::poseCallback(const geometry_msgs::Pose2DConstPtr& estimatedPose) const
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(estimatedPose->x, estimatedPose->y, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, estimatedPose->theta);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "Robot"));
}





#endif  //ROBOT_BASE
