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

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "timer");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob;

    // Declare you publishers and service servers
    ros::Publisher pubTime = nh_glob.advertise<std_msgs::Int32>("/NodeTime", 1) ;

    ros::Rate rate(1);   // Or other rate.
    double t0 = ros::Time::now().toSec();


    while (ros::ok()){
        ros::spinOnce();

        std_msgs::Int32 t;

        t.data = ros::Time::now().toSec() - t0;

        ROS_INFO_STREAM("time : " << t.data) ;

        pubTime.publish( t ) ;

        rate.sleep();
    }
}
