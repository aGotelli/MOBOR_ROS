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
 *        This node provides the initialization of all the ROS features in the
          function main. Here the initialization for the server is a little triky.
          Because once the server object is defined we need to feed the member function
          setCallback with our function as a parameter. Now the function that we want to
          pass is a memeber function of the class StaticFeedback_2_0. In order to pass
          the function we have to do the following:

          1)  Create an object that is defined as a pointer to a function of type
              void that takes as parameters mobile_robots_v2::StaticFeedbackConfig& and an int
              and here it is: boost::function< void (LocalConfig &, int) > f2;

          2)  Set this pointer to point at our function using boost::bind for returning it,
              but without any bounding. In fact there is:
              boost::bind( &StaticFeedback_2_0::SetParameters, &controller, _1, _2 )
              So first boost::bind takes the member function as reference,
              this is done by doing &StaticFeedback_2_0::SetParameters.
              the second parameter is the object of the class that contains the
              member function: &controller; also passed as reference.
              Then there are the other parameters that must not be binded so
              there is the definition _1 and _2 so the first is passed as first
              argument and the second as second argument.

          3)  The initialization is performed in a lambda function. However it could
              have been done simply in the code like this:
              dynamic_reconfigure::Server<LocalConfig>::CallbackType f;
              boost::function< void (LocalConfig &, int) > f2( boost::bind( &StaticFeedback_2_0::SetParameters, &controller, _1, _2 ) );
              f = f2;
              server.setCallback(f);
              However is overcolplicated

          4)  Actually, also the labda definition itself is unnecessary. It would
              been have sufficient to do:
              server.setCallback(boost::bind( &StaticFeedback_2_0::SetParameters, &controller, _1, _2 ));
              so pass the function and stop. But I wanted some extra passages in order to me more clear.
              In fact, boost::bind produces a function object that handle the one specified.

 */

#include "mobile_robots_v2/static_feedback_2_0.h"

typedef mobile_robots_v2::StaticFeedbackConfig LocalConfig;

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
