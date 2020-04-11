#include <mobile_robots/robot_2_0.h>




int main (int argc, char ** argv)
{

  ros::init(argc, argv, "Robot_2_0");

  const double L = 0.7;       //  [m]
  const double b = 0.5;       //  [m]
  const double wmax = 2.22;      //  [RAD/S]
  const double r = 0.04;     //  [m]

  Robot_2_0 robot(argc, argv, L, r, b, wmax);
//        Robot(_argc, _argv, _L, _r, _b, _wmax, _frameRate)

  robot.IsMoving();

  return 0;
}
