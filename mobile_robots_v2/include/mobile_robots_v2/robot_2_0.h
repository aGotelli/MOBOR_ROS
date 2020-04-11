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
 *        This file contains the needed overrides in order to implement the Odometry and
          the Callback for this type of robot.
          Additionally, this file contains the declaration of the Server element, necessary
          to perform the dynamic reconfigure. In the struct there is
          dynamic_reconfigure::Server<RobotConfig> server; that without the use of typedef:
          dynamic_reconfigure::Server<mobile_robots_v2::Robot_2_0Config> server;
          creates the Server element with the explicit specification of the data type.

          The setCallback member function of the class Server has to be initialized.
          In the constructor there is the Initialization. the usage of a lambda funtion is
          again of a good practise. In fact, the constructor is called when the Robot_2_0 element
          is created in the node file. So that at the Initialization the server must be configured.
          As a matter of fact, the main loop is in a memeber function of robot_base so there are no other way
          to set the server. (unless the creation of a new and unnecessary memeber function).

          In this file I wanted to create the callBack function as a memeber function.
          I could have declared it directly in the constructor with a lambda, as I did
          for the target node. However I wanted to speriment new ways to do the same thing.

          Here the member function must be passed as a parameter. In order to do that we can just
          use boost::bind without binding any parameter. In fact, the bind is performed using _1 and _2
          which means that both the two parameters are not binded and the function takes them when called
          from the outside (when called in the Server).

          The declaration is the following:

          boost::bind(&Robot_2_0::SetParameters, this, _1, _2);

          now as the function is a member function there is the need to define it as it is, a
          member function. So this is the reason for the declaration &Robot_2_0::SetParameters,
          because is a member function of the class Robot_2_0. thene there is a this.
          The boost bind takes ("reference to a member function of the class", "the class object", <---------->)
          but here the class object is the object for which the constructor is called, so the object is this.

          at this point the object that boost::bind defines is a an object that handle a void
          function. this function takes a RobotConfig parameter and an integer so:

          boost::function< void (RobotConfig &, int)> f2 = boost::bind(&Robot_2_0::SetParameters, this, _1, _2);


 *
 */



//CPP
#ifndef ROBOT_2_0_H
#define ROBOT_2_0_H
#include <mobile_robots_v2/robot_base.h>


//ROS
#include <mobile_robots_v2/Robot_2_0Config.h>


typedef mobile_robots_v2::Robot_2_0Config RobotConfig;

struct Robot_2_0 : public Robot
{

  Robot_2_0(int _argc, char** _argv, const double& _L, const double& _r, const double& _b, const double& _wmax, const double& _x=0, const double& _y=0, const double _theta=0, const int& _frameRate=100):
            Robot(_argc, _argv, _L, _r, _b, _wmax, _x, _y, _theta, _frameRate) {

              dynamic_reconfigure::Server<RobotConfig>::CallbackType f = [&] {

                boost::function< void (RobotConfig &, int)> f2 = boost::bind(&Robot_2_0::SetParameters, this, _1, _2);

                return f2; } ();

                server.setCallback(f);

            }

  void Odometry() override;

  void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) override;

  inline void SetParameters(RobotConfig& config, uint32_t level) {wmax = config.wmax; };

  dynamic_reconfigure::Server<RobotConfig> server;

};


void Robot_2_0::Odometry()
{
  v = (wl + wr)*r/2;
  w = r*(wl - wr)/(2*b);

  //compute the component of Velocities
  vx = v*cos(estimatedPose.theta);
  vy = v*sin(estimatedPose.theta);

  // update position
  estimatedPose.x += vx;
  estimatedPose.y += vy;
  estimatedPose.theta += w;

}


void Robot_2_0::TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist)
{

  //processing it as in mobile robot in ARPRO
  unsigned int alpha = 20;

  double v = _Twist->linear.x;
  double omega = _Twist->linear.y*alpha +_Twist->angular.z;

  wl = (v + b*omega)/r;
  wr = (v - b*omega)/r;

  double a = std::max(abs(wl)/wmax, abs(wr)/wmax) < 1 ? 1 : std::max(abs(wl)/wmax, abs(wr)/wmax);

  wl /= a;
  wr /= a;

}





#endif    //ROBOT_2_0_H
