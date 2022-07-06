#include "abidat_robot_control/base/forward_kinematic.h"
#include "abidat_robot_control/msg/detail/motor_control__struct.hpp"

//ROS
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

//ROS Messages
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/header.hpp>

// std lib
#include <memory>
#include <vector>

namespace abidat {
namespace robot {
namespace control {

class ForwardKinematicNode : public rclcpp::Node
{
public:
  // default constructor
  ForwardKinematicNode() : rclcpp::Node("ForwardKinematicNode") { }

  // default destructor
  ~ForwardKinematicNode() = default;

  /**
   * \brief Method that initializes the motor control publisher and the velocity subscriber
   */ 
  bool initialize();
  
  private:
  /**
   * \brief Method that will be called when a new twist message arrives
   * \param twist_msg the input messages to be received 
   */
  void callback(std::shared_ptr<geometry_msgs::msg::Twist> twist_msg);

  double distance_wheels_;
  double wheel_diameter_;

  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> velocity_subscriber_; //> subscriber for velocity
  std::vector<std::shared_ptr<rclcpp::Publisher<abidat_robot_control::msg::MotorControl>>> pubs_motor_control_; //> publisher for the motor control for each existing motor 
  std::shared_ptr<ForwardKinematics> forward_kinematics_; //> shared pointer declaration of type ForwardKinematics  
}; 

} //end namespace control
} //end namespace robot
} //end namespace abidat