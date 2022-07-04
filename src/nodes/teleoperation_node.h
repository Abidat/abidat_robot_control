/**
 * \brief      This class reads the controller-joystick parameters from the parameter server and stores them in the input_mapper_ object. 
 *             It publishes to /officerobot/cmd_vel the joy messages computed based on the activation function (from the input mapper) and on the data received from the controller.
 *  
 * \author     Claudia Bina (c.bina@abidat.de)
 * \author     Christian Wendt (c.wendt@abidat.de)
 * 
 * \file       teleoperation.h
 */
#pragma once

#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/joy__struct.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "library/input_mapping.h"
#include "get_keyboard_input.h"

namespace abidat {

namespace robot {

namespace control {


class TeleoperationNode 
{
public:
  TeleoperationNode();
  ~TeleoperationNode()
  {
    keyboard_input_.stop();
  }
  void switchInput(std::shared_ptr<const sensor_msgs::msg::Joy> joy);

private:
  /**
   * \brief Reads the parameters from the parameter server and maps them into the input mapper, It also sets the activation function.
   */
  bool readParameters();
  void joyCallback(std::shared_ptr<const sensor_msgs::msg::Joy> joy);
  void keyCallback(const int key);

  double currentReading;
  bool keyboard_enable_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy> sub_joy_;
  rclcpp::Publisher<geometry_msgs::msg::Twist> pub_twist_;

  InputMapping input_mapper_;
  GetKeyboardInput keyboard_input_{std::bind(&TeleoperationNode::keyCallback, this, std::placeholders::_1)};
};

} //end namespace control

} //end namespace robot

} //end namespace abidat
