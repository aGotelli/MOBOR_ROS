#include <QApplication>
#include <ros/ros.h>
#include "myviz.h"

int main(int argc, char **argv)
{

  ros::init( argc, argv, "myviz");

  QApplication app( argc, argv );

  MyViz* myviz = new MyViz();
  myviz->show();

  app.exec();

  delete myviz;
}
