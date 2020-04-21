/**
 * \file static StaticFeedback_2_0
 * \brief publish the results of a static feeedback controller for a (2, 0) robot
 * \author Andrea Gotelli
 * \version 0.1
 * \date 11 April 2020
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    ° "RobotPose"
 *    ° "TargetOdometry"
 *
 * Publishes to: <BR>
 *    ° "TwistToRobot"
 *
 * Description
 *
 */

#include "mobile_robots_v3/static_feedback_2_0.h"

typedef mobile_robots_v3::StaticFeedbackConfig LocalConfig;

int main(int argc, char** argv)
{
        ros::init(argc, argv, "StaticFeedback_2_0");

        StaticFeedback_2_0 controller;

        ros::NodeHandle controlNode;

        ros::Subscriber subtoTarget = controlNode.subscribe("TargetOdometry", 1, &Controller::UpdateTargetPosition, dynamic_cast<Controller*>( &controller ));

        ros::Subscriber subtoRobot = controlNode.subscribe("RobotPose", 1, &Controller::UpdateRobotPosition, dynamic_cast<Controller*>( &controller ));

        ros::Publisher twistPublisher = controlNode.advertise<geometry_msgs::Twist>("TwistToRobot", 1);


        //setting the controller frequency
        ros::Rate controllerFrameRate = 100;  //Hz

        //stardard suggested way
        dynamic_reconfigure::Server<LocalConfig> server;

        dynamic_reconfigure::Server<LocalConfig>::CallbackType f = [&] {

          boost::function< void (LocalConfig &, int) > f2( boost::bind( &StaticFeedback_2_0::SetParameters, &controller, _1, _2 ) );
          return f2;

        } ();


        server.setCallback(boost::bind( &StaticFeedback_2_0::SetParameters, &controller, _1, _2 ));




        while (ros::ok()) {

          ros::spinOnce();

          controller.ComputeCorrectionTwist();

          twistPublisher.publish(controller.TwistToRobot());

          controllerFrameRate.sleep();

        }


        return 0;
}
