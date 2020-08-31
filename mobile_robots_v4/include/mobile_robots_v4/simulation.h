/**
 * \file  simulator header file
 * \brief this file contains the initializaion of markers
 * \author  Andrea Gotelli
 * \version 0.1
 * \date  10 April 2020
 *
 * \param[in] none
 *
 * Subscribes to: <BR>
 *    °
 *
 * Publishes to: <BR>
 *    °
 *
 * Description
 *      In order to make the conde simplere this header file exists fo provide
        simple and fast initialization for the markers. in this way the node code
        remains lighter and simple.
 *
 */


//CPP
#ifndef SIMULATOR_H
#define SIMULATOR_H
#include <mobile_robots_v4/pose.h>

//ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace sim
{


struct Quaternion
{
  double w, x, y, z;
};

Quaternion ToQuaternion(double yaw, double pitch=0, double roll=0) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    const double cy = cos(yaw * 0.5);
    const double sy = sin(yaw * 0.5);
    const double cp = cos(pitch * 0.5);
    const double sp = sin(pitch * 0.5);
    const double cr = cos(roll * 0.5);
    const double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

struct EulerAngles {
    double roll, pitch, yaw;
};

EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}


void SetCoordinates(const Pose& _pose3D, visualization_msgs::Marker& _marker)
{
  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  _marker.pose.position.x = _pose3D.x;
  _marker.pose.position.y = _pose3D.y;
  _marker.pose.position.z = _pose3D.z;



  Quaternion q = ToQuaternion( _pose3D.theta); // yaw (Z), pitch (Y), roll (X)


  _marker.pose.orientation.x = q.x;
  _marker.pose.orientation.y = q.y;
  _marker.pose.orientation.z = q.z;
  _marker.pose.orientation.w = q.w;
}

void SetScale(const float& _sx, visualization_msgs::Marker& _marker)
{
  _marker.scale.x = _sx;
  _marker.scale.y = _sx;
  _marker.scale.z = _sx;
}

void SetColor(const float& _r, const float& _g, const float& _b, const float& _a, visualization_msgs::Marker& _marker)
{
  _marker.color.r = _r;
  _marker.color.g = _g;
  _marker.color.b = _b;
  _marker.color.a = _a;
}



visualization_msgs::Marker TargetMarkerInit()
{
  visualization_msgs::Marker targetMarker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  targetMarker.header.frame_id = "/map";
  targetMarker.header.stamp = ros::Time::now();


  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  targetMarker.ns = "targetShape";
  targetMarker.id = 0;


  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  targetMarker.type = visualization_msgs::Marker::SPHERE;


  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  targetMarker.action = visualization_msgs::Marker::ADD;


  SetScale(0.25, targetMarker);


  SetColor(1.0f, 0.0f, 0.0f, 0.7f, targetMarker);


  targetMarker.lifetime = ros::Duration();

  return targetMarker;
}


visualization_msgs::Marker RobotMarkerInit()
{
  visualization_msgs::Marker robotMarker;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  robotMarker.header.frame_id = "/map";
  robotMarker.header.stamp = ros::Time::now();


  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  robotMarker.ns = "RobotShape";
  robotMarker.id = 0;


  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  robotMarker.type = visualization_msgs::Marker::ARROW;


  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  robotMarker.action = visualization_msgs::Marker::ADD;


  SetScale(1.0, robotMarker);


  SetColor(0.0f, 1.0f, 0.0f, 1.0f, robotMarker);


  robotMarker.lifetime = ros::Duration();

  return robotMarker;

}


}










#endif //SIMULATOR_H
