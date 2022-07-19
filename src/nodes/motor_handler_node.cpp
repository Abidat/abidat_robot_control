/**
 * \brief      This class is used to controll the robot motors
 *  
 */
#include "motor_handler_node.h"
#include <atomic>
#include <cstdint>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace abidat {
namespace robot {
namespace control {

MotorHandlerNode::MotorHandlerNode()
  : rclcpp::Node("MotorHandlerNode")
  , brick_pi_(std::make_unique<BrickPi3>())
{

}

MotorHandlerNode::~MotorHandlerNode()
{
  resetBrickPiValues();
  is_running_ = false;

  if(state_thread_.joinable()) {
    state_thread_.join();
  }
  if(safety_thread_.joinable()) {
    safety_thread_.join();
  }
}

bool MotorHandlerNode::initialize()
{
  //Reading the parameters from launch filemotors_
  if(!readMotorConfigurations()) {
    RCLCPP_ERROR(get_logger(), "MotorHandlerNode: can't read motor configuration from ROS parameter server.");
    return false;
  }
  else {
    for(std::size_t motorIdx = 0; motorIdx < static_cast<std::size_t>(MotorPort::COUNT); ++motorIdx) {
      if(motor_configurations_[motorIdx].is_enabled == true) {
        //Initialize publisher motor_state and the subscriber motor_control for every enabled motor
        pub_motor_states_[motorIdx] = create_publisher<abidat_robot_control::msg::MotorState>(
          "/officerobot/motor_state_" + std::to_string(motorIdx),
          rclcpp::QoS(1).best_effort()
        );     
        sub_motor_controlls_[motorIdx] = create_subscription<abidat_robot_control::msg::MotorControl>(
          "/officerobot/motor_control_" + std::to_string(motorIdx),
          rclcpp::QoS(1).best_effort(),
          std::bind(&MotorHandlerNode::MotorHandlerNodeCallback, this, std::placeholders::_1, motorIdx)
        );
        RCLCPP_INFO(get_logger(), "The Publisher and the Subscriber for motor %i were started successfully", static_cast<int>(motorIdx));
      }
    }

    //Initialize Publisher Device_Info 
    pub_device_info_ = create_publisher<abidat_robot_control::msg::DeviceInfo>("/officerobot/device_info", rclcpp::QoS(1).best_effort());
    RCLCPP_INFO(get_logger(), "The Publisher for the BrickPi device info  was started successfully");  
  }

  if(this->initBrickPi() == false) {
    RCLCPP_ERROR(get_logger(), "MotorHandlerNode: initialization wasn't successful.");
    return false;
  }

  //safety
  //Start the thread that check the safety 
  safety_thread_ = std::thread(&MotorHandlerNode::TimeOutSafety, this);
  RCLCPP_INFO(get_logger(), "Safety thread was started successfully");
  
  RCLCPP_INFO(get_logger(), "---------- Configuration completed successfully ----------");
  return true;
}

bool MotorHandlerNode::readMotorConfigurations()
{
  std::array<std::string, static_cast<std::size_t>(MotorPort::COUNT)> MOTOR_PORT_NAMES = {"motor_a", "motor_b", "motor_c", "motor_d"};
  static_assert(MOTOR_PORT_NAMES.size() == 4u); // code below can only handle four motor ports

  // iterate over all motors and get parameters for each
  for (std::size_t motorIdx = 0; motorIdx < static_cast<std::size_t>(MotorPort::COUNT); ++motorIdx) {
    const std::string motor_port_name = MOTOR_PORT_NAMES[motorIdx];
    RCLCPP_INFO(get_logger(), "MotorHandlerNode: read parameter for motor %s. (motor index = %u)", motor_port_name.c_str(), static_cast<unsigned int>(motorIdx));
    MotorConfiguration motor_config;

    // declare all parameters that are used for parameterization
    // main parameter
    declare_parameter<bool>(motor_port_name + "/enable", motor_config.is_enabled);
    declare_parameter<bool>(motor_port_name + "/is_inverse", motor_config.is_inverse);
    declare_parameter<bool>(motor_port_name + "/is_servo", motor_config.is_servo);
    declare_parameter<std::int32_t>(motor_port_name + "/middle_position", motor_config.middle_position);

    // angle parameter
    declare_parameter<int>(motor_port_name + "/angle_limits/maximum", motor_config.angle_maximum_limit);
    declare_parameter<int>(motor_port_name + "/angle_limits/minimum", motor_config.angle_minimum_limit);

    // speed parameter
    declare_parameter<int>(motor_port_name + "/speed_limits/maximum", motor_config.speed_maximum_limit);
    declare_parameter<int>(motor_port_name + "/speed_limits/minimum", motor_config.speed_minimum_limit);

    // get all parameters
    // main parameter
    motor_config.is_enabled      = get_parameter(motor_port_name + "/enable").as_bool();
    motor_config.is_inverse      = get_parameter(motor_port_name + "/is_inverse").as_bool();
    motor_config.is_servo        = get_parameter(motor_port_name + "/is_servo").as_bool();
    motor_config.middle_position = get_parameter(motor_port_name + "/middle_position").as_int();
    motor_config.port = static_cast<MotorPort>(motorIdx);

    switch (static_cast<MotorPort>(motorIdx))
    {
    case MotorPort::PORTA:
      motor_config.port_id = PORT_A;
      break;

    case MotorPort::PORTB:
      motor_config.port_id = PORT_B;
      break;

    case MotorPort::PORTC:
      motor_config.port_id = PORT_C;
      break;

    case MotorPort::PORTD:
      motor_config.port_id = PORT_D;
      break;

    default:
      RCLCPP_FATAL(get_logger(), "MotorHandlerNode: unkown motor port. This never must happen! Skip configuration of current motor.");
      continue; // continue with next motor
    }

    RCLCPP_INFO(get_logger(), "Parameter %s is set to %s", (motor_port_name + "/enable").c_str(), motor_config.is_enabled ? "True" : "False");
    RCLCPP_INFO(get_logger(), "Parameter %s is set to %s", (motor_port_name + "/is_inverse").c_str(), motor_config.is_inverse ? "True" : "False");
    RCLCPP_INFO(get_logger(), "Parameter %s is set to %s", (motor_port_name + "/is_servo").c_str(), motor_config.is_servo ? "True" : "False");          
    RCLCPP_INFO(get_logger(), "Parameter %s is set to %d", (motor_port_name + "/middle_position").c_str(), motor_config.middle_position);

    // angle parameter
    motor_config.angle_maximum_limit = get_parameter(motor_port_name + "/angle_limits/maximum").as_int();
    motor_config.angle_minimum_limit = get_parameter(motor_port_name + "/angle_limits/maximum").as_int();
    motor_config.speed_maximum_limit = get_parameter(motor_port_name + "/speed_limits/maximum").as_int();
    motor_config.speed_minimum_limit = get_parameter(motor_port_name + "/speed_limits/minimum").as_int();

    // check if the set values for the motor port, angle- and speed limits are valid
    if (!motor_config.isValid()) {
      RCLCPP_FATAL(get_logger(), "Motor configuration of motor \"%s\" is invalid.", MOTOR_PORT_NAMES[motorIdx].c_str());
      return false;
    }
    motor_configurations_[motorIdx] = motor_config;
  }
  return true;
}

void MotorHandlerNode::MotorHandlerNodeCallback(const std::shared_ptr<abidat_robot_control::msg::MotorControl>& motor_control, const std::size_t motor_idx)
{
  // Store receive time for detection a timeout. Used in TimeOutSafety() method.
  stamp_last_motor_control_msg = get_clock()->now();

  std::int32_t position = motor_control->angle;
  std::int16_t speed    = motor_control->speed;

  // if Servo: the transferred angle  will be set
  if (motor_configurations_[motor_idx].is_servo == true && motor_configurations_[motor_idx].is_enabled) {
    if (motor_control->angle > motor_configurations_[motor_idx].angle_maximum_limit) {
      position = motor_configurations_[motor_idx].angle_maximum_limit;
      RCLCPP_ERROR(get_logger(), "The transferred angle must be smaller than %d",  motor_configurations_[motor_idx].angle_maximum_limit);
    }
    if (motor_control->angle < motor_configurations_[motor_idx].angle_minimum_limit) {  
      position = motor_configurations_[motor_idx].angle_minimum_limit;
      RCLCPP_ERROR(get_logger(), "The transferred angle must be larger than %d",  motor_configurations_[motor_idx].angle_minimum_limit);
    }

    brick_pi_->set_motor_position(motor_configurations_[motor_idx].port_id, position);
  }
    
  // if Motor is in normal mode (wheel mode), the velocity in [deg/sec]m will be set
  if (motor_configurations_[motor_idx].is_servo == false && motor_configurations_[motor_idx].is_enabled) {
    if (motor_control->speed > motor_configurations_[motor_idx].speed_maximum_limit) {
      speed = motor_configurations_[motor_idx].speed_maximum_limit;
      RCLCPP_ERROR(get_logger(), "The transferred velocity must be smaller than %d",  motor_configurations_[motor_idx].speed_maximum_limit);
    }
    if (motor_control->speed < motor_configurations_[motor_idx].speed_minimum_limit) {
      speed = motor_configurations_[motor_idx].speed_minimum_limit;
      RCLCPP_ERROR(get_logger(), "The transferred velocity must be higher than %d",  motor_configurations_[motor_idx].speed_minimum_limit);
    }

    brick_pi_->set_motor_dps( motor_configurations_[motor_idx].port_id, ( motor_configurations_[motor_idx].is_inverse ? speed : (-1) * speed ));
  }
}

bool MotorHandlerNode::initBrickPi()
{
  // Make sure that the BrickPi3 is communicating and that the firmware is compatible with the drivers.
  // Note: Library will return 0 if no errors detected. (#define ERROR_NONE 0)
  if(brick_pi_->detect() != 0) {
    RCLCPP_ERROR(get_logger(), "MotorHandlerNode: BrickPi is not connected succesfully.");
    return false;
  } 
  else {
    RCLCPP_INFO(get_logger(), "MotorHandlerNode: BrickPi is connected succesfully");
    // the buffer variables are needed to 

    for(auto& motor : motor_configurations_) {
      // skip configuration if motor isn't enabled
      if(!motor.is_enabled) {
        continue;
      }
      
      if (brick_pi_->reset_motor_encoder(motor.port_id) == 1) { // int error = 1
        RCLCPP_ERROR(get_logger(), "MotorHandlerNode: error occurred during initialization of motor %d.", motor.port);
        return false;
      }
      else { 
        RCLCPP_INFO(get_logger(), "MotorHandlerNode: encoders for port: %d are resetted to zero.", motor.port);
      }
    }                     
  }
     
  // Start the thread that publishes the state.
  // @todo a ROS timer would be better.
  state_thread_ = std::thread(&MotorHandlerNode::readBrickPiValues, this);
  RCLCPP_INFO(get_logger(), "MotorHandlerNode: BrickPi state publisher thread was started succesfully");
  return true;
}

void MotorHandlerNode::resetBrickPiValues()
{
  // Reset everything so there are no run-away motors
  //std::lock_guard<std::mutex>guard(mutex_brick_access_);
  brick_pi_->reset_all();
}

void MotorHandlerNode::readBrickPiValues()
{
  //std::lock_guard<std::mutex> guard(mutex_brick_access_);
  while(is_running_ && rclcpp::ok())
  {
    //Publish Motor States
    for(std::size_t motorIdx = 0; motorIdx < static_cast<std::size_t>(MotorPort::COUNT); ++motorIdx) {
      if(motor_configurations_[motorIdx].is_enabled == true) { //maybe instead of  check is.enable, check if the publisher was started 
        abidat_robot_control::msg::MotorState motor_state_msg;
        std_msgs::msg::Header header_;

        header_.stamp = get_clock()->now();
        header_.frame_id = "base_link_frame_id";
        motor_state_msg.header = header_;

        brick_pi_->get_motor_status(
          motor_configurations_[motorIdx].port_id,
          motor_states_[motorIdx].state, 
          motor_states_[motorIdx].power,
          motor_states_[motorIdx].position,
          motor_states_[motorIdx].deg_per_sec
        );
        
        motor_state_msg.state    = motor_states_[motorIdx].state;
        motor_state_msg.power    = motor_states_[motorIdx].power;
        motor_state_msg.position = motor_states_[motorIdx].position;
        motor_state_msg.dps      = motor_states_[motorIdx].deg_per_sec; 
        
        pub_motor_states_[motorIdx]->publish(motor_state_msg);
      }
    }

    //Publish Device info 
    abidat_robot_control::msg::DeviceInfo device_info_msg;
    std_msgs::msg::Header header_;

    header_.stamp = get_clock()->now();
    header_.frame_id = "base_link_frame_id";
    
    char info_[33]; // Room for the 32-character serial number string plus the NULL terminator. @todo use a constant from brick pi header instead!

    device_info_msg.header = header_;

    brick_pi_->get_manufacturer(info_);
    device_info_msg.manufacturer = std::string(info_);

    brick_pi_->get_board(info_);
    device_info_msg.board = std::string(info_);

    brick_pi_->get_id(info_);
    device_info_msg.serial_number =  std::string(info_);
    
    brick_pi_->get_version_hardware(info_);
    device_info_msg.hardware_version = std::string(info_);

    brick_pi_->get_version_firmware(info_);
    device_info_msg.firmware_version = std::string(info_);

    device_info_msg.battery_voltage = std::to_string(brick_pi_->get_voltage_battery());
    device_info_msg.voltage_9v  = std::to_string(brick_pi_->get_voltage_9v());
    device_info_msg.voltage_5v  = std::to_string(brick_pi_->get_voltage_5v());
    device_info_msg.voltage_3v3 = std::to_string(brick_pi_->get_voltage_3v3());

    pub_device_info_->publish(device_info_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void MotorHandlerNode::TimeOutSafety()
{
  while (is_running_ == true) {
    for(std::size_t motorIdx = 0; motorIdx < static_cast<std::size_t>(MotorPort::COUNT); ++motorIdx) {
      safety_mutex_.lock();

      //Stops the robot (set speed to Zero ), if the last receiving massages is older than 0.3s
      if (((get_clock()->now() - stamp_last_motor_control_msg).seconds() > watchdog_timeout) 
           && motor_configurations_[motorIdx].is_enabled == true
           && (motor_configurations_[motorIdx].is_servo == false)) {
        brick_pi_->set_motor_dps(motor_configurations_[motorIdx].port_id, 0);
      }
        
      //checks if the BrickPi can connect to the Motor (check if the connection is still working), if not it gives a warning that the motor isn't connect
      if ((motor_states_[motorIdx].state == 2 )&&(motor_configurations_[motorIdx].is_enabled == true)) {
        RCLCPP_ERROR(get_logger(), "The connection to motor %d is lost!", static_cast<int>(motorIdx));
      }

      safety_mutex_.unlock();
    }
  }
}

} //end namespace control
} //end namespace robot
} //end namespace abidat


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto motor_handle = std::make_shared<abidat::robot::control::MotorHandlerNode>();
  motor_handle->initialize();

  rclcpp::spin(motor_handle);  

  return 0;
}

