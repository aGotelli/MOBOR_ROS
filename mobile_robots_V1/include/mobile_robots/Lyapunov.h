#ifndef LYAPUNOV_H
#define LYAPUNOV_H

#include <mobile_robots/control_base.h>

#include <math.h>       // sinh


struct Lyapunov_2_0 : public Controller
{
  Lyapunov_2_0(const double& _Kx, const double& _Ky, const double& _Ktheta) :
          Controller(_Kx, _Ky, _Ktheta) {}

  //perform the control
  void ComputeCorrectionTwist() override;

};



void Lyapunov_2_0::ComputeCorrectionTwist()
{
  //here a good example of imediately invoked lambda initializers to be const correct an simplify initializations

  const double xe =  cos(robotPose.theta)*(targetPose.x - robotPose.x) + sin(robotPose.theta)*(targetPose.y - robotPose.y);
  const double ye = -sin(robotPose.theta)*(targetPose.x - robotPose.x) + cos(robotPose.theta)*(targetPose.y - robotPose.y);

  double theta_e = atan2(targetPose.vy, targetPose.vx) - robotPose.theta;
  theta_e = theta_e - 2*M_PI*floor((theta_e + M_PI)/(2*M_PI));


  //const double v_r = targetPose.vx*cos(robotPose.theta) + targetPose.vx*sin(robotPose.theta);

  const double v_r = targetPose.vx*cos(theta_e) + targetPose.vx*sin(theta_e);

  const double vrThreshold = 0.01;
  const double temp = targetPose.vx*targetPose.ay - targetPose.ax*targetPose.vy;

  double omega_r = 0;
  if(v_r > vrThreshold)
    omega_r = temp / (v_r*v_r);


  const double v = v_r*cos(theta_e) + Kx*xe;

  const double omega = omega_r + Ky*ye*v_r*sinh(theta_e) + Ktheta*theta_e;


  twistToRobot.linear.x = v;
  twistToRobot.angular.z = omega;
}




/*
void Lyapunov_2_0::ComputeCorrectionTwist()
{
  //here a good example of imediately invoked lambda initializers to be const correct an simplify initializations

  const double xe =  cos(robotPose.theta)*(targetPose.x - robotPose.x) + sin(robotPose.theta)*(targetPose.y - robotPose.y);
  const double ye = -sin(robotPose.theta)*(targetPose.x - robotPose.x) + cos(robotPose.theta)*(targetPose.y - robotPose.y);

  const double theta_e = [&] {
                  const double temp = atan2(targetPose.vy, targetPose.vx) - robotPose.theta;

                  return temp - 2*M_PI*floor((temp + M_PI)/(2*M_PI));
                  } ();



  const double v_r = targetPose.vx*cos(robotPose.theta) + targetPose.vx*sin(robotPose.theta);

  //const double v_r = targetPose.vx*cos(theta_e) + targetPose.vx*sin(theta_e);

  const double omega_r = [&] {
                const double temp = targetPose.vx*targetPose.ay - targetPose.ax*targetPose.vy;

                const double threshold = 0.1;

                return v_r  < threshold  ? 0 : temp / (v_r*v_r);
                } ();

  const double v = v_r*cos(theta_e) + Kx*xe;

  const double omega = [&] {

                const double temp = targetPose.vx*targetPose.ay - targetPose.ax*targetPose.vy;

                const double threshold = 0.1;

                return abs(theta_e) < threshold ? omega_r + Ky*ye*v_r : omega_r + Ky*ye*v_r*sin(theta_e)/theta_e + Ktheta*theta_e;
                } ();

  twistToRobot.linear.x = v;
  twistToRobot.angular.z = omega;
}
*/














#endif // LYAPUNOV_H
