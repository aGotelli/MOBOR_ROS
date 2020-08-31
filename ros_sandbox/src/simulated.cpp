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
#include <std_msgs/Int32.h>

void callBack(std_msgs::Int32 time)
{
  ROS_INFO_STREAM("s_time : " << time.data ) ;
}

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "simulated_timer");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob;

    // Declare you publishers and service servers
    ros::Subscriber pubTime = nh_glob.subscribe<std_msgs::Int32>("/NodeTime", 1, callBack) ;

    ros::Rate rate(1);   // Or other rate.


    while (ros::ok()){
        ros::spinOnce();

        rate.sleep();
    }
}
