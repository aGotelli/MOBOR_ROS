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

#include <mobile_robots_v4/pose.h>


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

  Robot(const double& _x, const double& _y, const double _theta, const int& _frameRate);

  virtual void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) const = 0;    //different robot may interpretate the Twist in different ways

  virtual void IsMoving() const;

  virtual void UpdateTF() const = 0;

protected:
  //node tools
  mutable ros::NodeHandle robotNode;
  mutable ros::Subscriber desiredTwist, tfRobot;
  mutable ros::Publisher robotPosition;
  mutable tf::TransformBroadcaster tfBroadcaster;

  struct RobotTransforms
  {
    RobotTransforms(const tf::Transform& _transform,
                    const std::string& _parent,
                    const std::string& _children) : transform(_transform),
                                                    parent(_parent),
                                                    children(_children) {}

    RobotTransforms operator=(const tf::Transform& _transform)
    {
      this->transform = _transform;

      return* this;
    }


    tf::Transform transform;
    std::string parent;
    std::string children;

  };

  mutable std::vector<RobotTransforms> robotTransforms;

  mutable double v, w;
  mutable double vx, vy;
  mutable double delta_x, delta_y, delta_theta;

  //frame rate of the robot
  mutable ros::Rate robotFrameRate;

  //actual robot position that will be published
  mutable geometry_msgs::Pose2D robotPose;

  // Handling the time
  mutable ros::Time currentTime, prevTime;
  mutable ros::Duration timeElapsed;
};



Robot::Robot(const double& _x, const double& _y, const double _theta, const int& _frameRate):
delta_x(0),
delta_y(0),
delta_theta(0),
v(0),
w(0),
vx(0),
vy(0),
robotFrameRate(ros::Rate(_frameRate)),  //  [Hz]
prevTime(ros::Time::now())
{

  ROS_INFO_STREAM(ros::this_node::getName() << "online...");

  desiredTwist = robotNode.subscribe("TwistToRobot", 1, &Robot::TwistToProcess, this);

  robotPosition = robotNode.advertise<geometry_msgs::Pose2D>("RobotPose", 1);

  robotPose.x = _x;                    //  [m]
  robotPose.y = _y;                    //  [m]
  robotPose.theta = _theta;            //  [RAD]

}


void Robot::IsMoving() const
{

  double t0, t;

  while (ros::ok()) {

    t0 = ros::Time::now().toSec() ;

    ros::spinOnce();

		robotPosition.publish(robotPose);

    UpdateTF();

    for(const auto& transf : robotTransforms ) {
      tfBroadcaster.sendTransform(tf::StampedTransform(transf.transform, ros::Time::now(), transf.parent, transf.children));
    }

    t = ros::Time::now().toSec() - t0;

    ROS_INFO_STREAM("time elapsed : " << t << " [s]") ;

		robotFrameRate.sleep();

  }

}

#endif  //ROBOT_BASE
