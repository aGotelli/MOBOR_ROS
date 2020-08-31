#ifndef POLYGON_LOADER_H
#define POLYGON_LOADER_H


#include "pluginlib_tutorials/polygon_base.h"
#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <xmlrpcpp/XmlRpcException.h>











class PolygonLoader {
public:

  PolygonLoader();




private:

  pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader;

  boost::shared_ptr<polygon_base::RegularPolygon> triangle;

  boost::shared_ptr<polygon_base::RegularPolygon> square;

  ros::NodeHandle nh_;


};




PolygonLoader::PolygonLoader() : poly_loader("pluginlib_tutorials", "polygon_base::RegularPolygon")
{

  // std::string plugins_param;
  // std::string plugins_namespace = "/";
  // if(!nh_.searchParam(plugins_namespace, plugins_param)) {
  //   const std::string& ns = nh_.getNamespace();
  //   ROS_ERROR("PolygonLoader: failed to locate plugins namespace '%s', "
  //     "search started from '%s'", plugins_namespace.c_str(), ns.c_str());
  //   return;
  // }
  //
  // ROS_INFO("PolygonLoader: Loading plugins from '%s'", plugins_param.c_str());
  //
  // // this is used to get the list of available plugins
  // XmlRpc::XmlRpcValue plugins;
  // if(!nh_.getParam(plugins_param, plugins)) {
  //   ROS_ERROR("Failed to get list of plugins");
  //   return;
  // }
  //
  // if(plugins.size() == 0) {
  //   ROS_ERROR("No plugins found");
  //   return;
  // }
  //
  // for(const auto& plugin : plugins) {
  //   ROS_INFO_STREAM("found : " << plugin.first << " and : " << plugin.second );
  // }

  try
  {

    triangle = poly_loader.createInstance("polygon_plugins::Triangle");
    triangle->initialize(10.0);

    square = poly_loader.createInstance("polygon_plugins::Square");
    square->initialize(10.0);

    ROS_INFO("Triangle area: %.2f", triangle->area());
    ROS_INFO("Square area: %.2f", square->area());

    triangle->a = param("a", 0);

    ROS_INFO_STREAM("T area : " << triangle->a << "S area : " << square->a );
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }
}



#endif  //  POLYGON_LOADER_H
