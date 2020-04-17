/**
 * \brief      This class reads the controller-joystick parameters from the parameter server and stores them in the input_mapper_ object. 
 *             It publishes to /officerobot/cmd_vel the joy messages computed based on the activation function (from the input mapper) and on the data received from the controller.
 *  
 * \author     Claudia Bina (c.bina@abidat.de)
 * 
 * \file       teleoperation.h
 */


#ifndef ___TELEOPERATION_H___
#define ___TELEOPERATION_H___

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>

#include "input_mapping.h"
#include "get_keyboard_input.h"

namespace officerobot {
  
class TeleoperationNode 
{
public:
  TeleoperationNode();
  ~TeleoperationNode()
  {
    keyboard_input_.stop();
  }
  void switchInput(const sensor_msgs::Joy::ConstPtr& joy);

private:
  /**
   * \brief Reads the parameters from the parameter server and maps them into the input mapper, It also sets the activation function.
   */
  bool readParameters();
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void keyCallback(const int key);

  double currentReading;
  bool keyboard_enable_ = false;

  ros::Subscriber sub_joy_;
  ros::Publisher pub_twist_;
  ros::NodeHandle nh;

  InputMapping input_mapper_;
  GetKeyboardInput keyboard_input_{std::bind(&TeleoperationNode::keyCallback, this, std::placeholders::_1)};
};

}

#endif

