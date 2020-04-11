#include "mobile_robots/simulator.h"






//only needed for calling the callBack function in the easiest way
Pose* ptrRobotPose;

//take the published robot pose
void GetRobotPose(const geometry_msgs::Pose2D::ConstPtr& _robotPose)
{

  (* ptrRobotPose) = (* _robotPose);

}

//take the published target pose
void GetTargetPose(const mobile_robots::Reference::ConstPtr& _targetPose, Pose* _ptrTargetPose)
{

  (*_ptrTargetPose) = (*_targetPose);

}

//allowing another access
void GetTargetPose2(const mobile_robots::Reference::ConstPtr& _targetPose, Pose& targetPose)
{
  //recall the first function as this second one is just for trying new
  //calling methods and boost::bind
  GetTargetPose(_targetPose, &targetPose);
}



int main(int argc, char** argv)
{

	ros::init(argc, argv, "simulator");

	ros::NodeHandle simulatorNode;

  ros::Rate simulatorFrameRate(100);

  /*
  the previus line is the same as the two below, but careful with the equation
        ros::Rate simulatorFrameRate = 100;
  the previus equation was automatically doing what is called implicit conversion
  so basically it was expecting a Rate object but we passed a int, so the compiler
  figures it out that actually can transform the integer as follow
        ros::Rate simulatorFrameRate = ros::Rate(100);
  or maibe doing something like
        ros::Rate simulatorFrameRate(100)
  */

	ros::Publisher pubTargetPose = simulatorNode.advertise<visualization_msgs::Marker>("TargetToRviz", 1);
	ros::Publisher pubRobotPose = simulatorNode.advertise<visualization_msgs::Marker>("RobotToRviz", 1);


  //setting up the position of both robot and target that will be published bu the node
	Pose targetPose, robotPose;

  //setting up the pointers in order to change the position by a non very smart callback fuction calling.
  Pose* ptrTargetPose = &targetPose;
  ptrRobotPose = &robotPose;


  //this is another way to use the received message binding the callback function to the parameters (as the geometry_msgs::POint is passed
  //as first argument I have putted the flag _1 in the boost::bind function).
  //i could have used just a normal global variable... but where is the fun then?
	ros::Subscriber subToTarget = simulatorNode.subscribe<mobile_robots::Reference>("TargetReference", 1, boost::bind(GetTargetPose, _1, ptrTargetPose));
  //ros::Subscriber subToTarget = simulatorNode.subscribe<mobile_robots::Reference>("Target_Position", 2, boost::bind(GetTargetPose2, _1, boost::ref(targetPose)));
  /*
  The previus two method were explayining an important feature of C++,
  so basically the bind will, actually, bind one parameter of the passed function
  the parameter binded is expressed with a flag(_1, _2 ecc) that represents the
  parameter position in the function declaration. However the second parameter is passed as a copy.
  This makes important the use of boost::ref() in order to not copy the object
  but keeping the referece passing
  the version that uses the pointer is ok as there are no problem in
  coping a pointer, as the result of changin the parameter is still the same
  */

	ros::Subscriber subToRobot = simulatorNode.subscribe("RobotPose", 1, GetRobotPose);

  visualization_msgs::Marker targetMarker, robotMarker;


  sim::ConfigureMarkers(targetMarker, robotMarker);

	while (ros::ok())	{

    ros::spinOnce();  //needed for update from callback function

		sim::SetCoordinates(targetPose, targetMarker);

    sim::SetCoordinates(robotPose, robotMarker);

    pubTargetPose.publish(targetMarker);

    pubRobotPose.publish(robotMarker);

		simulatorFrameRate.sleep();
	}



	return 0;
}
