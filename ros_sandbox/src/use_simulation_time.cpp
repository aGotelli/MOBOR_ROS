/**
 * \file
 * \brief
 * \author
 * \version 0.1
 * \date
 *
 * \param[in]
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
 *
 */


//Cpp
#include <sstream>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <stdlib.h>
#include <math.h>

//ROS
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <rosgraph_msgs/Clock.h>



ros::Time t;
ros::Time t0;

bool take_first = true;
void callBack(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  t = msg->clock;
  if( take_first ) {
    t0 = msg->clock;
    take_first = false ;
  }
}

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "use_sim_time");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob;

    // Declare you publishers and service servers
    ros::Subscriber pubTime = nh_glob.subscribe<rosgraph_msgs::Clock>("/clock", 1, callBack) ;

    ros::Rate rate(500);   // Or other rate.

    bool initialized = false;



    while (ros::ok()){
        ros::spinOnce();



        ROS_INFO_STREAM("time : " << (t - t0).toSec() );

        rate.sleep();
    }
}
