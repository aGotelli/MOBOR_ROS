/**
 * \file  circular target node
 * \brief this node publishes a simple reference to follow
 * \author  Andrea Gotelli
 * \version 0.1
 * \date  12 April 2020
 *
 * \param[in] none
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    ° "TargetOdometry" topic that contains the target position and velocities
 *      the massage wil be processed by the controllers in order to move the robot.
 *
 *    ° "TargetToRviz" a visualization marker to rviz that is useful to see the
 *      target moving in kind of real time
 *
 * Description
 *
 *
 */

//CPP
#include <mobile_robots_v3/simulation.h>
#include <mobile_robots_v3/pose.h>



//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <mobile_robots_v3/TargetConfig.h>
#include <tf/transform_broadcaster.h>


//here a typedef to meke the following more readable
typedef mobile_robots_v3::TargetConfig ConfigType;
typedef dynamic_reconfigure::Server<ConfigType>::CallbackType TargetCallBackType;

void poseCallback(const nav_msgs::OdometryConstPtr& targetOdometry)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(targetOdometry->pose.pose.position.x, targetOdometry->pose.pose.position.y, 0.0) );
  tf::Quaternion q(
  targetOdometry->pose.pose.orientation.x,
  targetOdometry->pose.pose.orientation.y,
  targetOdometry->pose.pose.orientation.z,
  targetOdometry->pose.pose.orientation.w );
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "Circulartarget"));
}


int main (int argc, char** argv)
{

	//ROS Initialization
  ros::init(argc, argv, "Circulartarget");

  ros::NodeHandle targetNode;

  ros::Publisher targetPublisher = targetNode.advertise<nav_msgs::Odometry>("TargetOdometry", 1);
  ros::Publisher targetToRviz = targetNode.advertise<visualization_msgs::Marker>("TargetToRviz", 1);

  ros::Subscriber tfSub = targetNode.subscribe("TargetOdometry", 1, &poseCallback);

  nav_msgs::Odometry targetOdometry;

  ros::Rate targetFrameRate(100);

  //the radius of the circle
  const double radius = [&] {

    double r;
    targetNode.param("radius", r, 3.0);
    return r; } ();

  //the target angulat velocity
  double omega;

  //intializing the time
  const double t0 = ros::Time::now().toSec();

  //fix the target z coordinate
  targetOdometry.pose.pose.position.z = 1;

  //set the reference to whom the coordinates are published
  //targetOdometry.header.frame_id = "map";

  //Here the declaration of the Server element.
  dynamic_reconfigure::Server<ConfigType> server;

  //the callback function for the dynamic reconfigure
  TargetCallBackType lambdaCallBack = [&] (ConfigType config, uint32_t level) {
    omega = config.omega;
  };

  //now the server callback can be setted by pasing the correct function.
  server.setCallback(lambdaCallBack);

  //initialize the marker for the target
  auto targetMarker = sim::TargetMarkerInit();

  while(ros::ok()) {

        ros::spinOnce();

        const double t = ros::Time::now().toSec() - t0;

        targetOdometry.pose.pose.position.x = radius*cos(omega*t);
        targetOdometry.pose.pose.position.y = radius*sin(omega*t);

        sim::Quaternion q = sim::ToQuaternion(omega*t);

        targetOdometry.pose.pose.orientation.x = q.x;
        targetOdometry.pose.pose.orientation.y = q.y;
        targetOdometry.pose.pose.orientation.z = q.z;
        targetOdometry.pose.pose.orientation.w = q.w;

        targetOdometry.twist.twist.linear.x = -omega*radius*sin(omega*t);
        targetOdometry.twist.twist.linear.y = omega*radius*cos(omega*t);

        targetPublisher.publish(targetOdometry);

        sim::SetCoordinates(Pose(radius*cos(omega*t), radius*sin(omega*t), 1), targetMarker);

        targetToRviz.publish(targetMarker);


        targetFrameRate.sleep();
    }

    return 0;
}
