#include <mobile_robots/Lyapunov.h>







int main(int argc, char** argv)
{
  ros::init(argc, argv, "Lyapunov_2_0");

  ros::NodeHandle controlNode;
  const double Kx = 10;
  const double Ky = 10;
  const double Ktheta = 10;

  Lyapunov_2_0 controller(Kx, Ky, Ktheta);

  ros::Subscriber subToRobot = controlNode.subscribe("RobotPose", 1, &Controller::UpdateRobotPosition, dynamic_cast<Controller*>( &controller ));

  ros::Subscriber subToTarget = controlNode.subscribe("TargetReference", 1, &Controller::UpdateTargetPosition, dynamic_cast<Controller*>( &controller ));

  ros::Publisher twistPublisher = controlNode.advertise<geometry_msgs::Twist>("TwistToRobot", 1);

  ros::Rate controllerFrameRate(100);

  while(ros::ok()) {

    ros::spinOnce();

    controller.ComputeCorrectionTwist();

    twistPublisher.publish(controller.TwistToRobot());

    controllerFrameRate.sleep();
  }
}
