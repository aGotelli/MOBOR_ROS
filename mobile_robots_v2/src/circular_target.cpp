/**
 * \file  circular target node
 * \brief this node publishes a simple reference to follow
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
 *    ° "TargetOdometry" topic that contains the target position and velocities
 *      the massage wil be processed by the controllers in order to move the robot.
 *
 *    ° "TargetToRviz" a visualization marker to rviz that is useful to see the
 *      target moving in kind of real time
 *
 * Description
 *      The aim here is to use a simple node to see the very basic setting for the
        dynamic_reconfigure. In fact, there will be initialized the server, that is
        the variable, or parameter or element, that takes into account the reconfigure
        and perform it when a parameter is changed from the rqt_reconfigure console.
        the server is element of the class Server that is inside the dynamic_reconfigure
        namespace. This class takes value type that is an TargetConfig type that is itself
        an UDT that is inside the mobile_robots_v2 namespace (check devel/include/mobile_robots_v2).
        In fact below there is the Class declaration and evrithing we need to know.


        The element the type Server (called with a lot of fantasy "server") calls a Server
        member function that is Server::setCallback(boost::function<void(ConfigType &, uint32_t level)>)
        so this member function takes a functions as argument. The function is defined as usually in boost::function
        It is a void function that takes a ConfigType (templete parameter) and a uint32_t level.
        Oh! it is actually our Callback function!!

        So the only thing we need to do is to feed this memeber function with our callBack function.

        the procedure will be done using a lambda function but other procedure will be explained.
 *
 */

//CPP
#include <mobile_robots_v2/simulation.h>
#include <mobile_robots_v2/pose.h>



//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <mobile_robots_v2/TargetConfig.h>


int main (int argc, char** argv)
{

	//ROS Initialization
  ros::init(argc, argv, "Circulartarget");

  ros::NodeHandle targetNode;

  ros::Publisher targetPublisher = targetNode.advertise<nav_msgs::Odometry>("TargetOdometry", 1);
  ros::Publisher targetToRviz = targetNode.advertise<visualization_msgs::Marker>("TargetToRviz", 1);


  nav_msgs::Odometry targetOdometry;

  ros::Rate targetFrameRate(100);

  //the radius of the circle
  const double radius = 4;

  //the target angulat velocity
  double omega;

  //intializing the time
  const double t0 = ros::Time::now().toSec();

  //fix the target z coordinate
  targetOdometry.pose.pose.position.z = 1;

  //here a typedef to meke the following more readable
  typedef mobile_robots_v2::TargetConfig ConfigType;
  typedef dynamic_reconfigure::Server<ConfigType>::CallbackType TargetCallBackType;


//Here the declaration of the Server element. It has the value type explicited as
//ConfigType that is: mobile_robots_v2::TargetConfig.
  dynamic_reconfigure::Server<ConfigType> server;

//As explained there is the need to create a callBack funtion that has to be passed
//as parameter into the Server::setCallback(const CallbackType &callback) member function
  TargetCallBackType lambdaCallBack = [&] (ConfigType config, uint32_t level) {
    omega = config.omega;
  };

  //now the server callback can be setted by pasing the correct function.
  server.setCallback(lambdaCallBack);

  /*
  Here the utility to declare a lambda funtion that is basically a variable
  that acts like a function (the next step of a pointer ot a function)

  In this way is super easy to pass the correct function to the member function.


  Another way could be as the following:

  declare a function as usually (before the main)

  void Callback(ConfigType& config, uint32_t level)
  {
    do things...
  }

  then the function can be passed as element using boost::bind but without any bounding.

  boost::bind(Callback, _1, _2); which in my advice does not make much sense...

  then the element that takes the (un)binded function is a element that refers to a void
  function which takes a ConfigType element and a int level so:
  boost::function<void (ConfigType&, int)> funtion; = boost::bind(Callback, _1, _2);

  the funtion that has to be passed is defined as member funtion
  dynamic_reconfigure::Server<ConfigType>::CallbackType funtionToPass = funtion_;

  server.setCallback(funtionToPass);

  but for me is overcomplicated.

  it is also possible to do directly

  server.setCallback(funtion_);


  */



  //initialize the marker for the target
  auto targetMarker = sim::TargetMarkerInit();

  while(ros::ok()) {

        ros::spinOnce();

        const double t = ros::Time::now().toSec() - t0;

        targetOdometry.pose.pose.position.x = radius*cos(omega*t);
        targetOdometry.pose.pose.position.y = radius*sin(omega*t);

        targetOdometry.twist.twist.linear.x = -omega*radius*sin(omega*t);
        targetOdometry.twist.twist.linear.y = omega*radius*cos(omega*t);

        targetPublisher.publish(targetOdometry);

        sim::SetCoordinates(Pose(radius*cos(omega*t), radius*sin(omega*t), 1), targetMarker);

        targetToRviz.publish(targetMarker);


        targetFrameRate.sleep();
    }

    return 0;
}




/*


        namespace dynamic_reconfigure
        {

        template <class ConfigType>
        class Server
        {
        public:

          typedef boost::function<void(ConfigType &, uint32_t level)> CallbackType; <--1-->

          void setCallback(const CallbackType &callback)
          {
            boost::recursive_mutex::scoped_lock lock(mutex_);
            callback_ = callback; <--2-->

            callCallback(config_, ~0); // At startup we need to load the configuration with all level bits set. (Everything has changed.)
            updateConfigInternal(config_);
          }

          <--3-->

        private:
          ros::NodeHandle node_handle_;
          ros::ServiceServer set_service_;
          ros::Publisher update_pub_;
          ros::Publisher descr_pub_;
          <--4-->
          CallbackType callback_;
          ConfigType config_;
          ConfigType min_;
          ConfigType max_;
          ConfigType default_;
          <--5-->
          boost::recursive_mutex &mutex_;
          boost::recursive_mutex own_mutex_; // Used only if an external one isn't specified.
          bool own_mutex_warn_;


              void callCallback(ConfigType &config, int level)
          {
            if (callback_) // At startup we need to load the configuration with all level bits set. (Everything has changed.)
              try {
                callback_(config, level);
              }
              catch (std::exception &e)
              {
                ROS_WARN("Reconfigure callback failed with exception %s: ", e.what());
              }
              catch (...)
              {
                ROS_WARN("Reconfigure callback failed with unprintable exception.");
              }
            else
              ROS_DEBUG("setCallback did not call callback because it was zero."); /// @todo kill this line.
          }

          <--6-->
          <--7-->

        };
        }





        <--1--> So the CallbackType is a new type defined by the creators of the class
                the C++ translation is the functor that takes a ConfigType element (template class)
                and the uint32_t level, that are the parmaters of the callback itself

        <--2--> In the private part of the Server class there is the definition CallbackType callback_;

        <--3--> Here the explaination of what happens when we set the callback it is saved in the class and
                passed to the function callCallback;

        <--4--> Here we can see that the class works more or less like a node. It publishes topics but
                uses a ServiceServer.

        <--5--> here we can see the usage of a template class. In fact, the class is declared in a general way.
                So it can take any paramere we pass with the parameter list, or explicit type specifier <...>
                This Class is defined as tamplate with the template parameter ConfigType. In the code we
                explicitly specify the data type when declaring the element:
                dynamic_reconfigure::Server<ConfigType> server; that is, without the use of the typedef
                dynamic_reconfigure::Server<mobile_robots_v2::TargetConfig> server;
                So inside the class the elements will be:
                boost::function<void(mobile_robots_v2::TargetConfig &, uint32_t level)> callback_;
                mobile_robots_v2::TargetConfig config_;
                ...
                ..
                .
                and that is the explicit definition for the class member.


        <--5--> This last function basically give it a try to the callback at the beginning.

        <--7--> There are other functions and parameters but the ones before were the only ones of interst
                here. because everithing else is just for the recursively check and call during the ros::ok()
*/
