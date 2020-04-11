/**
 * \file robot node
 * \brief code for a moving robot
 * \author  Andrea Gotelli
 * \version 0.1
 * \date  6 April 2020
 *
 * \param[in] robot max speed
 *
 * Subscribes to: <BR>
 *    ° "TargetOdometry"
 *
 * Publishes to: <BR>
 *    ° "RobotPose"
 *
 * Description
 *        The behavior of the robot node is to descrive the mathematic of a (2, 0) robot
          moving with the convential differential drive configuration
 *
 */



//ROS
#include <mobile_robots_v2/robot_2_0.h>



int main(int argc, char** argv)
{

    ros::init(argc, argv, "Robot_2_0");

    const double L = 0.7;       //  [m]
    const double b = 0.5;       //  [m]
    const double wmax = 2.22;      //  [RAD/S]
    const double r = 0.04;     //  [m]

    Robot_2_0 robot(argc, argv, L, r, b, wmax);

    robot.IsMoving();

    return 0;

}
