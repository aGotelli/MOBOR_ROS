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
    ros::init(argc, argv, "publisher");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob;

    // Declare you publishers and service servers
    ros::Publisher pubTime = nh_glob.advertise<std_msgs::Int32>("/Counter", 1) ;

    ros::Duration(0.5).sleep();

    ros::Rate rate(5);   // Or other rate.
    double t0 = ros::Time::now().toSec();
    int index = 0;
    while (ros::ok()){
        ros::spinOnce();

        std_msgs::Int32 num;

        num.data = index;

        ROS_INFO_STREAM("number : " << num.data) ;

        pubTime.publish( num ) ;

        index ++;

        rate.sleep();
    }
}
