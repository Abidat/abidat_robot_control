/**
 * \brief      This node publishes and subscribes twist messages for the movement of the omniwheel robot "marvin"
 * 
 * \author     Egzone Ademi (e.ademi@abidat.de)
 * \maintainer Egzone Ademi (e.ademi@abidat.de)
 */
#include "forward_kinematic_node.h"
#include "abidat_robot_control/base/forward_kinematic_calculation.h"
#include "abidat_robot_control/base/forward_kinematic.h"

#include "abidat_robot_control/msg/motor_control.hpp" 
#include <functional>
#include <memory>
#include <rclcpp/utilities.hpp>

namespace abidat {
namespace robot {
namespace control {

bool ForwardKinematicNode::initialize()
{
  RCLCPP_INFO(get_logger(), "Start setting the Robot parameters");

  // Getting needed parameters
  declare_parameter<double>("distance_wheels", 0.0);
  declare_parameter<double>("wheel_diameter", 0.0);

  distance_wheels_ = get_parameter("distance_wheels").as_double();
  wheel_diameter_  = get_parameter("wheel_diameter").as_double();

  // check if values are in a valid range 
  if(!(distance_wheels_ > 0.0 && wheel_diameter_ > 0.0))
  {
    RCLCPP_ERROR(get_logger(), "The wheel distance and the wheel diameter must be larger than 0"); 
    return false;
  }

  // TODO: make forward kinematics selectable using a factory
  forward_kinematics_ = std::make_shared<ForwardKinematics>(distance_wheels_, wheel_diameter_);

  // Initialize publisher
  for(std::size_t i = 0; i < forward_kinematics_->getNumMotors(); ++i) {
    const std::string topic_name = "/officerobot/motor_control_" + std::to_string(i);
    RCLCPP_INFO(get_logger(), "ForwardKinematic: advertise publisher \"%s\".", topic_name.c_str());
    pubs_motor_control_.push_back(create_publisher<abidat_robot_control::msg::MotorControl>(topic_name, rclcpp::QoS(1).best_effort()));
  }

  // Initialize subscriber
  velocity_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
    "/officerobot/cmd_vel",
    rclcpp::QoS(1).best_effort(),
    std::bind(&ForwardKinematicNode::callback, this, std::placeholders::_1)
  );

  return true;
}

void ForwardKinematicNode::callback(std::shared_ptr<geometry_msgs::msg::Twist> twist_msg){
  
  // get total movement from forward kinematics
  std::vector<double> velocity;

  velocity = forward_kinematics_->calculateForwardKinematics(*twist_msg);
  
  abidat_robot_control::msg::MotorControl motor_control_msg;
  // Todo: Change message type to twist stamp and use header from this message instead
  motor_control_msg.header.stamp = get_clock()->now();
  motor_control_msg.header.frame_id = "base_link";

  for(std::size_t i = 0; i < velocity.size(); i++)
  {
    motor_control_msg.speed = static_cast<std::int16_t>(convertRADToRPM(velocity[i]));
    pubs_motor_control_[i]->publish(motor_control_msg);
  }
}

} //end namespace control

} //end namespace robot

} //end namespace abidat

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto kinematic = std::make_shared<abidat::robot::control::ForwardKinematicNode>();

  if (!kinematic->initialize())
    return 1;

  // execute nodes
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(kinematic);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
