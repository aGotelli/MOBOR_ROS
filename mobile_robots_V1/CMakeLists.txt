cmake_minimum_required(VERSION 3.14.4)
project(mobile_robots)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Debug)




## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  message_generation #in order to use my messages types
  tf2_ros

)

## System dependencies are found with CMake's conventions




## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Twist2D.msg
   Pose.msg
   Reference.msg
   Velocities.msg
   Accelerations.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)



###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include src
  LIBRARIES mobile_robots
    CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
	roscpp
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  msg
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ library
add_library(mobile_robots
#            src/targets/circular_target.cpp
#            src/simulator.cpp
              staticfeedback_2_0.cpp
      )

target_link_libraries(mobile_robots ${catkin_LIBRARIES} )
add_dependencies(mobile_robots ${${PROJECT_NAME}_EXPORTED_TARGETS})


#####################
## Circular Target ##
#####################
## Declare a C++ executable
add_executable(circular_target src/targets/circular_target.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(circular_target mobile_robots)


############################
## Static Feedback (2, 0) ##
############################
## Declare a C++ executable
add_executable(staticfeedback_2_0 staticfeedback_2_0.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(staticfeedback_2_0 mobile_robots)





#####################
## Lyapunov (2, 0) ##
#####################
## Declare a C++ executable
add_executable(lyapunov_2_0 Lyapunov_2_0.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(lyapunov_2_0 mobile_robots)




    ###############
    ## Simulator ##
    ###############
## Declare a C++ executable
add_executable(simulator src/simulator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(simulator ${catkin_LIBRARIES})




  ##################
  ## Robot (2, 0) ##
  ##################
## Declare a C++ executable
add_executable(robot_2_0 src/robots/robot_2_0.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_2_0 ${catkin_LIBRARIES})





    ##############
    ## Rviz GUI ##
    ##############
# Declare a C++ executable
#add_executable(rviz_gui rviz/rviz_conf.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(rviz_gui ${catkin_LIBRARIES})






# Declare a C++ executable
#add_executable(temp_robot_2-0 src/robots/temp_robot.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(temp_robot_2-0 ${catkin_LIBRARIES})
