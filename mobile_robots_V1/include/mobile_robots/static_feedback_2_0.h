#ifndef STATIC_FEEDBACK_2_0_H
#define STATIC_FEEDBACK_2_0_H

#include <mobile_robots/control_base.h>




class StaticFeedback_2_0 : public Controller
{
public:

  StaticFeedback_2_0(const double _d, const double _Kp):
              Controller(_d, _Kp) {}


  void UpdateTargetPosition(const mobile_robots::Reference::ConstPtr& _targetPose) override;

  void UpdateRobotPosition(const geometry_msgs::Pose2D::ConstPtr& _robotPose) override;

  void ComputeCorrectionTwist() override;


};


void StaticFeedback_2_0::UpdateTargetPosition(const mobile_robots::Reference::ConstPtr& _targetPose)
{
  //need to pass the actual object not the pointer
  targetPose = (*_targetPose);
}


void StaticFeedback_2_0::UpdateRobotPosition(const geometry_msgs::Pose2D::ConstPtr& _robotPose)
{
  //ROS_INFO_STREAM("gettin some signals from a robot..." << _estimatedPose->pose.x << " " << _estimatedPose->pose.y);
  robotPose = (*_robotPose);
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

/*
void StaticFeedback_2_0::ComputeCorrectionTwist()
{
  //robot and reference positions an velocities
  const auto xy_p = [&] {
              Eigen::VectorXd temp(2);

              temp(0) = robotPose.x + d*cos(robotPose.theta);
              temp(1) = robotPose.y + d*sin(robotPose.theta);

              return temp;
  } ();

  //using a lambda function to be const-correct
  const auto K = [&] {
        Eigen::Matrix2d M;

        M << cos(robotPose.theta),  -d*sin(robotPose.theta),
             sin(robotPose.theta),   d*cos(robotPose.theta);

        return M;
  } ();

  const auto zc_dot = targetPose.xy_dot() + Kp*(targetPose.xy() - xy_p);

  const auto v_omega = K.inverse()*zc_dot;

  twistToRobot.linear.x = v_omega[0];
  twistToRobot.angular.z = v_omega[1];

}
*/















#endif //STATIC_FEEDBACK_2_0_H
