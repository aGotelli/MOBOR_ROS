#include <ros/ros.h>										//main library
#include <mobile_robots/Reference.h>				//needed for pubblishing position
#include <ros/console.h>								//needed for the cout


#include <math.h>											//I need M_PI

#include <mobile_robots/pose.h>


int main(int argc, char** argv)
{

	//same name for all the target, they will kick eachother off
	//if multiple call happens
	ros::init(argc, argv, "Target");

	ROS_INFO_STREAM("Target Online.");
	//Create and hande to process the node
	ros::NodeHandle targetNode;

	ros::NodeHandle localNode("~");
	//ros::NodeHandle localNode("VipArea");
	//ros::Publisher positionPublisher = localNode.advertise<mobile_robots::Reference>("TargetReference", 1);

	
	//the Publisher object allows to publish messages on the specified topic
	ros::Publisher positionPublisher = targetNode.advertise<mobile_robots::Reference>("TargetReference", 1);


	//setting up the target frequency
	ros::Rate targetFrequency(100);


	//Setting up geometrical parameters
	const Pose Center(0, 0, 1);
	float radius;

	if( localNode.getParam("radius", radius) )
		localNode.setParam("radius", radius);
	else
		localNode.param("radius", radius, 8.0f);


	float omega;

	if( localNode.getParam("omega", omega) )
		localNode.setParam("omega", omega);
	else
		localNode.param("omega", omega, (float)M_PI/3);


	//time handling section
	double t;

	//current time
	const double t0 = ros::Time::now().toSec();

	//position message
	mobile_robots::Reference targetRef;

	targetRef.pose.z = Center.z;

	while (ros::ok()) {


		//take the data
		ros::spinOnce();


		//current time
		t = ros::Time::now().toSec() - t0;


		//set the velocity
		// if( localNode.getParam("/circular_target/omega", omega) )
		// 	localNode.setParam("/circular_target/omega", omega);


		//store the reference
		targetRef.pose.x = Center.x + radius*cos(omega*t);
		targetRef.pose.y = Center.y +radius*sin(omega*t);

		targetRef.dot_pose.vx = -radius*sin(omega*t)*omega;
		targetRef.dot_pose.vy = radius*cos(omega*t)*omega;

		targetRef.ddot_pose.ax = -radius*cos(omega*t)*omega*omega;
		targetRef.ddot_pose.ay = -radius*sin(omega*t)*omega*omega;


		//send the reference
		positionPublisher.publish(targetRef);


		//wait...
		targetFrequency.sleep();
	}

	return 0;
}
