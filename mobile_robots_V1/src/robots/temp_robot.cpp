#include <ros/ros.h>



#include <geometry_msgs/Twist.h>    //the received Twist
#include <geometry_msgs/Pose2D.h>

#include <string.h>

#include <mobile_robots/pose.h>



double b = 0.5;
double r = 0.02;
double wmax = 10;

void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist , double& wl, double& wr)
{

  //processing it as in mobile robot in ARPRO
  unsigned int alpha = 20;

  double _v = _Twist->linear.x;
  double _omega = _Twist->linear.y*alpha +_Twist->angular.z;

  wl = (_v + b*_omega)/r;
  wr = (_v - b*_omega)/r;

  if(abs(wl) > wmax || abs(wr) > wmax) {
      //ROS_INFO_STREAM("Velocities are too high!");
  }

  double a = std::max(abs(wl)/wmax, abs(wr)/wmax) < 1 ? 1 : std::max(abs(wl)/wmax, abs(wr)/wmax);

  wl /= a;
  wr /= a;


  //ROS_INFO_STREAM("wl = " << wl << " wr = " << wr);


 }


int main(int argc, char** argv)
{
  ros::init(argc, argv,  "Robot_2_0");

  ros::NodeHandle robotNode;

  double wl = 0;
  double wr = 0;

  ros::Subscriber desiredTwist = robotNode.subscribe<geometry_msgs::Twist>("TwistToRobot", 1,
                                                                        boost::bind(TwistToProcess, _1, boost::ref(wl),  boost::ref(wr)));


  ros::Rate robotFrameRate(100);

  ros::Publisher position = robotNode.advertise<geometry_msgs::Pose2D>("RobotPose", 10);

  double v, w, vx, vy;

  geometry_msgs::Pose2D robotPose;
  while(ros::ok())
  {

      v = (wl + wr)*r/2;
      w = r*(wl - wr)/(2*b);

      //compute the component of Velocities
      vx = v*cos(robotPose.theta);
      vy = v*sin(robotPose.theta);

      // update position
      robotPose.x += vx;
      robotPose.y += vy;
      robotPose.theta += w;

      position.publish(robotPose);

      ros::spinOnce();

      robotFrameRate.sleep();

  }
  return 0;
}
