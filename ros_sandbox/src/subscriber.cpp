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

std::vector<int> v;
void callBack(std_msgs::Int32 num)
{
  v.push_back( num.data );
}

int main (int argc, char** argv)
{

	//ROS Initialization
    ros::init(argc, argv, "subscriber");

    // Define your node handles: YOU NEED AT LEAST ONE !
    ros::NodeHandle nh_glob;

    // Declare you publishers and service servers
    ros::Subscriber pubTime = nh_glob.subscribe<std_msgs::Int32>("/Counter", 10, callBack) ;

    ros::Rate rate(1);   // Or other rate.


    while (ros::ok()){
        ros::spinOnce();
        ROS_INFO_STREAM("In the main loop" ) ;

        for(const int& i : v) {
          ROS_INFO_STREAM( i );
        }

        v.clear();

        rate.sleep();
    }
}
