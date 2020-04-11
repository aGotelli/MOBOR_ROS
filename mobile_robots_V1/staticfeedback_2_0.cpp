#include "mobile_robots/static_feedback_2_0.h"


//the controller node is both a listener and a publisher as it will recieve the
//robot and target positions and pubblih the movement the robot should do in order to
//follow the target


int main(int argc, char** argv)
{
        ros::init(argc, argv, "StaticFeedback_2_0");

        const double d = 0.05;
        const double Kp = 5;

        StaticFeedback_2_0 controller(d, Kp);

        ros::NodeHandle controlNode;
        //ros::NodeHandle localNode("VipArea");
        //ros::Subscriber subtoTarget = localNode.subscribe("TargetReference", 1, &StaticFeedback_2_0::UpdateTargetPosition, &controller );

        ros::Subscriber subtoTarget = controlNode.subscribe("TargetReference", 1, &StaticFeedback_2_0::UpdateTargetPosition, &controller );

        ros::Subscriber subtoRobot = controlNode.subscribe("RobotPose", 1, &StaticFeedback_2_0::UpdateRobotPosition, &controller );

        ros::Publisher twistPublisher = controlNode.advertise<geometry_msgs::Twist>("TwistToRobot", 1);


        //setting the controller frequency
        ros::Rate controllerFrameRate = 100;  //Hz


        while (ros::ok()) {

          ros::spinOnce();

          controller.ComputeCorrectionTwist();

          twistPublisher.publish(controller.TwistToRobot());

          controllerFrameRate.sleep();

        }


        return 0;
}
