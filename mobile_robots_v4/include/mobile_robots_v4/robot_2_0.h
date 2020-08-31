/**
 * \file  (2,0) robot header file
 * \brief this file contains the model implementation of a (2,0) robot
 * \author  Andrea Gotelli
 * \version 0.1
 * \date  21 April 2020
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
 *    The node contains the needed function in order to simulate the behaviur of
      a (2, 0) robot. The computations take into account the time, obtaining the time
      dispacemente at each time interval. The fact that the frame rate is setted to
      1000 Hz makes the simulation very close to a real time simulation.
      Addictionally, the computations update the wheels angle.

      Sensor are implemented as server. The aim in that every localization algorith can ask
      the wheels angle and will have the answer back.
 *
 */



//CPP
#ifndef ROBOT_2_0_H
#define ROBOT_2_0_H
#include <mobile_robots_v4/robot_base.h>


//ROS


struct GeneralizedCoordinates
{

  GeneralizedCoordinates(const double& _x=0, const double& _y=0, const double& _theta=0,
                        const double& _phi_1f=0, const double& _phi_2f=0,
                        const double& _phi_3c=0, const double& _beta_3c=0) :
                        x(_x), y(_y), theta(_theta),
                        phi_1f(_phi_1f), phi_2f(_phi_2f),
                        phi_3c(_phi_3c), beta_3c(_beta_3c)  {}

  GeneralizedCoordinates operator=(const Eigen::VectorXd& res)
  {
    this->x = res(0);
    this->y = res(1);
    this->theta = res(2);
    // this->phi_1f = res(3);
    // this->phi_2f = res(4);
    // this->phi_3c = res(5);
    // this->beta_3c = res(6);

    return* this;
  }


  void Integration(const GeneralizedCoordinates& q_dot, const ros::Duration& timeElapsed)
  {
    x += q_dot.x*timeElapsed.toSec();    //  [m]
    y += q_dot.y*timeElapsed.toSec();    //  [m]
    theta += q_dot.theta*timeElapsed.toSec(); //  [RAD]

  }

  double x, y, theta;

  double phi_1f, phi_2f, phi_3c;

  double beta_3c;
};



class Robot_2_0 : public Robot
{

public:
  Robot_2_0(const double& _L, const double& _b, const double& _d, const double& _r=0.05, const double& _wmax=10, const double& _x=0, const double& _y=0, const double _theta=0, const int& _frameRate=1000):
            r(_r),                                  //  [m]
            L(_L),                                  //  [m]
            b(_b),                                  //  [m]
            d(_d),                                  //  [m]
            wmax(_wmax),
            q( GeneralizedCoordinates(_x, _y, _theta) ),
            q_dot( GeneralizedCoordinates() ),
            Robot(_x, _y, _theta, _frameRate) {

              tf::Quaternion quat;

              // declaration of the transform
              tf::Transform RearJoint;
              tf::Transform MovingPlatform;
              tf::Transform wheel_1;
              tf::Transform wheel_2;

              //  Transformation initialization
              MovingPlatform.setOrigin( tf::Vector3( q.x, q.y, 0.0) );
              quat.setRPY(0, 0, q.theta);
              MovingPlatform.setRotation(quat);

              //  Transformation initialization
              RearJoint.setOrigin( tf::Vector3(-b, 0, 0) );
              quat.setRPY(0, 0, M_PI);
              RearJoint.setRotation(quat);

              //  Transformation initialization
              wheel_1.setOrigin( tf::Vector3(0, -L, 0) );
              quat.setRPY(0, 0, 0);
              wheel_1.setRotation(quat);

              //  Transformation initialization
              wheel_2.setOrigin( tf::Vector3(0, L, 0) );
              quat.setRPY(0, 0, 0);
              wheel_2.setRotation(quat);


              robotTransforms.push_back(RobotTransforms(MovingPlatform, "map", "moving_plat") );
              robotTransforms.push_back(RobotTransforms(RearJoint, "moving_plat", "rear_joint") );
              robotTransforms.push_back(RobotTransforms(wheel_1, "moving_plat", "wheel_1") );
              robotTransforms.push_back(RobotTransforms(wheel_2, "moving_plat", "wheel_2") );
            }


  // The way the robot processes the twist
  void TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) const override;

  void UpdateTF() const override;


private:

  mutable GeneralizedCoordinates q, q_dot;

  //radius and track gauge of the wheels
  const double L, b, d;
  const double r;
  const double wmax;

};



void Robot_2_0::TwistToProcess(const geometry_msgs::Twist::ConstPtr& _Twist) const
{

  const auto u = [&] {

    // the input
    double wl = (_Twist->linear.x + b*_Twist->angular.z)/r;   //  [RAD/s]
    double wr = (_Twist->linear.x - b*_Twist->angular.z)/r;   //  [RAD/s]

    double scalingFactor = std::max(abs(wl)/wmax, abs(wr)/wmax) < 1 ? 1 : std::max(abs(wl)/wmax, abs(wr)/wmax);

    wl /= scalingFactor;   //  [RAD/s]
    wr /= scalingFactor;   //  [RAD/s]

    return Eigen::Vector2d((wl + wr)*r/2, r*(wl - wr)/(2*b));

  } ();

  Eigen::MatrixXd J(3, 2);
  J << cos(q.theta),  0,
       sin(q.theta),  0,
            0      ,  1;


  q_dot = J*u;

  //compute the displacement
  currentTime = ros::Time::now();
  timeElapsed = currentTime - prevTime;
  prevTime = currentTime;

  delta_x = q_dot.x*timeElapsed.toSec();    //  [m]
  delta_y = q_dot.y*timeElapsed.toSec();    //  [m]
  delta_theta = q_dot.theta*timeElapsed.toSec(); //  [RAD]

  // update position
  robotPose.x += delta_x;
  robotPose.y += delta_y;
  robotPose.theta += delta_theta;

  q.Integration(q_dot, timeElapsed);

}

void Robot_2_0::UpdateTF() const
{
  tf::Transform transform;
  tf::Quaternion quat;
  transform.setOrigin( tf::Vector3(q.x, q.y, 0.0) );
  quat.setRPY(0, 0, q.theta);
  transform.setRotation(quat);
  robotTransforms[0] = transform;

}



#endif    //ROBOT_2_0_H
