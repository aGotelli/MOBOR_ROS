/**
 * \file  static feedback(2, 0) header file
 * \brief this file contains the static feedback implementation for a (2, 0) robot
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
 *        This file contains the function overrides necessary to create a new controller
          from the Controller base class. Additionally, there is the implementation of
          the parameter callBack function as a member function.
 *
 */

//CPP
#ifndef STATIC_FEEDBACK_2_0_H
#define STATIC_FEEDBACK_2_0_H

#include <mobile_robots_v2/control_base.h>
#include "mobile_robots_v2/StaticFeedbackConfig.h"


class StaticFeedback_2_0 : public Controller
{
public:

  StaticFeedback_2_0():
              Controller() {}


  void ComputeCorrectionTwist() override;

  //dynamic reconfigure for the parameters
  void SetParameters(mobile_robots_v2::StaticFeedbackConfig& config, uint32_t level);

};


void StaticFeedback_2_0::SetParameters(mobile_robots_v2::StaticFeedbackConfig& config, uint32_t level)
{
  d = config.d;
  Kp = config.Kp;

}



void StaticFeedback_2_0::ComputeCorrectionTwist()
{
  //robot and reference positions an velocities
  Eigen::VectorXd xy_p(2), xy_ref(2), xy_ref_dot(2);

  xy_p[0] = robotPose.x + d*cos(robotPose.theta);
  xy_p[1] = robotPose.y + d*sin(robotPose.theta);

  xy_ref[0] = targetPose.x;
  xy_ref[1] = targetPose.y;

  xy_ref_dot[0] = targetPose.vx;
  xy_ref_dot[1] = targetPose.vy;


  const auto zc_dot = xy_ref_dot + Kp*(xy_ref - xy_p);

  //using an imediatly invoked lambda initializer.
  const auto K = [&] {
        Eigen::Matrix2d M;

        M << cos(robotPose.theta),  -d*sin(robotPose.theta),
             sin(robotPose.theta),   d*cos(robotPose.theta);

        return M;
  } ();

  const auto v_omega = K.inverse()*zc_dot;

  twistToRobot.linear.x = v_omega[0];
  twistToRobot.angular.z = v_omega[1];

}

#endif //STATIC_FEEDBACK_2_0_H
