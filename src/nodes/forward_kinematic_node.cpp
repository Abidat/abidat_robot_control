/**
 * \brief      This node publishes and subscribes twist messages for the movement of the omniwheel robot "marvin"
 * 
 * \author     Egzone Ademi (e.ademi@abidat.de)
 * \maintainer Egzone Ademi (e.ademi@abidat.de)
 */
#include "forward_kinematic_node.h"
#include "forward_kinematic_calculation.h"
#include "forward_kinematic.h"

#include "abidat_robot_control/MotorControl.h"

namespace officerobot {

void ForwardKinematicNode::initialize(ros::NodeHandle node, ros::Subscriber velocity_subscriber) {

  // Initialize publisher
  for(std::size_t i = 0; i < forward_kinematics_->getNumMotors(); ++i)  
    pub_motor_control_[i] = node.advertise<abidat_robot_control::MotorControl>("/officerobot/motor_control_" + i, 1);
  
  // Initialize subscriber
  velocity_subscriber = node.subscribe<geometry_msgs::Twist>("/officerobot/cmd_vel", 1, &ForwardKinematicNode::callback, this);
}

bool ForwardKinematicNode::readParams(){

  ROS_INFO("Start setting the Robot parameters");

  //Setup private node handler
  ros::NodeHandle private_nh("~");

  // Getting needed parameters
  if(private_nh.hasParam("distance_wheels"))
  {
    private_nh.getParam("distance_wheels", this->distance_wheels_);
    ROS_INFO("Parameter distance_wheels is set to %f", this->distance_wheels_);
  }

  if(private_nh.hasParam("wheel_diameter"))
  {
    private_nh.getParam("wheel_diameter", this->wheel_diameter_);
    ROS_INFO("Parameter wheel_diameter is set to %f", this->wheel_diameter_);
  }

  // check if values are in a valid range
  if(distance_wheels_ > 0.0 && wheel_diameter_ > 0.0)
  {
    return true;
  }

  else
  {
    ROS_ERROR("The wheel distance, the wheel diameter and the distance of the axes must be larger than 0");
    return false;
  }
  // TODO: make forward kinematics selectable using a factory
  forward_kinematics_ = std::make_shared<officerobot::OfficeRobotForwardKinematics>(distance_wheels_, wheel_diameter_);
}

void ForwardKinematicNode::callback(const geometry_msgs::TwistConstPtr& twist_msg){
  
  // get total movement from forward kinematics
  std::vector<double> velocity;

  velocity = forward_kinematics_->calculateForwardKinematics(*twist_msg);
  
  abidat_robot_control::MotorControl motor_control_msg;
  // Todo: Change message type to twist stamp and use header from this message instead
  motor_control_msg.header.stamp = ros::Time::now();
  motor_control_msg.header.frame_id = "base_link";

  for(std::size_t i = 0; i < velocity.size(); i++)
  {
    motor_control_msg.speed = static_cast<std::int16_t>(convertRADToRPM(velocity[i]));
    pub_motor_control_[i].publish(motor_control_msg);
  }
}

} //end namespace officerobot

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ForwardKinematicNode");
    officerobot::ForwardKinematicNode officeRobot;
    ros::spin();

    return 0;
}