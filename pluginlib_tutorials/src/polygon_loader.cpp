#include "pluginlib_tutorials/polygon_loader.h"





int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_the_pluginlub");

  // pluginlib::ClassLoader<polygon_base::RegularPolygon> poly_loader("pluginlib_tutorials", "polygon_base::RegularPolygon");
  //
  //
  // try
  // {
  //   boost::shared_ptr<polygon_base::RegularPolygon> triangle = poly_loader.createInstance("polygon_plugins::Triangle");
  //   triangle->initialize(10.0);
  //
  //   boost::shared_ptr<polygon_base::RegularPolygon> square = poly_loader.createInstance("polygon_plugins::Square");
  //   square->initialize(10.0);
  //
  //   ROS_INFO("Triangle area: %.2f", triangle->area());
  //   ROS_INFO("Square area: %.2f", square->area());
  // }
  // catch(pluginlib::PluginlibException& ex)
  // {
  //   ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  // }

  PolygonLoader loader_;


  return 0;
}
