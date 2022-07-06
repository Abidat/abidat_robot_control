/**
 * \brief      This class is used to controll the robot motors
 *  
 */
#include "motor_handler_node.h"
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>

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
    pub_device_info = nh.advertise<abidat_robot_control::DeviceInfo>("/officerobot/device_info", 1);
    ROS_INFO("The Publisher for the BrickPi device info  was started successfully");  
  }

  if(this->initBrickPi() == false)
  {
    ROS_ERROR("MotorHandlerNode: initialization wasn't successfull.");
    return false;
  }
  //safety
  //Start the thread that check the safety 
  safety_thread_ = std::thread(&MotorHandlerNode::TimeOutSafety, this);
  ROS_INFO("Safety thread was started successfully");
  
  ROS_INFO("---------- Configuration completed successfully ----------");
  return true;
}

bool MotorHandlerNode::readMotorConfigurations(ros::NodeHandle& privNh)
{
  std::array<std::string, static_cast<std::size_t>(MotorPort::COUNT)> motorPortNames = {"motor_a", "motor_b", "motor_c", "motor_d"};
  assert(motorPortNames.size() == 4); // to prefent nasty configuration if someone forgets to a new port name

  for (std::size_t motorIdx = 0; motorIdx < static_cast<std::size_t>(MotorPort::COUNT); ++motorIdx)
  {
    const std::string motorPortName = motorPortNames[motorIdx];
    MotorConfiguration motorConfig;

    // gets all required parameters for each motor if there is wanted (exists inside the launch as parameter)
    if(privNh.hasParam(motorPortName))
    {
      ROS_INFO("MotorHandlerNode: initialize motor %s.", motorPortName.c_str());

      if (privNh.hasParam(motorPortName + "/enable"))
      {
        privNh.getParam(motorPortName + "/enable", motorConfig.is_enabled);
        ROS_INFO("Parameter %s is set to %s", (motorPortName + "/enable").c_str(), motorConfig.is_enabled ? "True" : "False");
      }
      if (privNh.hasParam(motorPortName + "/is_inverse"))
      {
        privNh.getParam(motorPortName + "/is_inverse", motorConfig.is_inverse);
        ROS_INFO("Parameter %s is set to %s", (motorPortName + "/is_inverse").c_str(), motorConfig.is_inverse ? "True" : "False");
      }
      if (privNh.hasParam(motorPortName + "/is_servo"))
      {
        privNh.getParam(motorPortName + "/is_servo", motorConfig.is_servo);
        ROS_INFO("Parameter %s is set to %s", (motorPortName + "/is_servo").c_str(), motorConfig.is_servo ? "True" : "False");          
      }
      if (privNh.hasParam(motorPortName + "/middle_position")) 
      {
        privNh.getParam(motorPortName + "/middle_position", motorConfig.middle_position);
        ROS_INFO("Parameter %s is set to %d", (motorPortName + "/middle_position").c_str(), motorConfig.middle_position);
      }

      motorConfig.port = static_cast<MotorPort>(motorIdx);

      switch (static_cast<MotorPort>(motorIdx))
      {
      case MotorPort::PORTA:
        motorConfig.port_id = PORT_A;
        break;

      case MotorPort::PORTB:
        motorConfig.port_id = PORT_B;
        break;

      case MotorPort::PORTC:
        motorConfig.port_id = PORT_C;
        break;

      case MotorPort::PORTD:
        motorConfig.port_id = PORT_D;
        break;

      default:
        ROS_FATAL("MotorHandlerNode: unkown motor port. This never must happen! Skip configuration of current motor.");
        continue; // continue with next motor
      }
    }

    if(privNh.hasParam(motorPortName + "/angle_limits"))
    {
      if(privNh.hasParam(motorPortName + "/angle_limits/maximum"))
      {
        privNh.getParam(motorPortName + "/angle_limits/maximum", motorConfig.angle_maximum_limit);
        ROS_INFO("Parameter %s is set to %d", (motorPortName + "angle_limits/maximum").c_str(), motorConfig.angle_maximum_limit);
      }
            
      if(privNh.hasParam(motorPortName + "/angle_limits/minimum"))
      {
        privNh.getParam(motorPortName + "/angle_limits/minimum", motorConfig.angle_minimum_limit);
        ROS_INFO("Parameter %s is set to %d", (motorPortName + "/angle_limits/minimum").c_str(), motorConfig.angle_minimum_limit);
      }
    }

    if(privNh.hasParam(motorPortName + "/speed_limits"))
    {
      if(privNh.hasParam(motorPortName + "/speed_limits/maximum"))
      {
        privNh.getParam(motorPortName + "/speed_limits/maximum", motorConfig.speed_maximum_limit);
        ROS_INFO("Parameter %s is set to %d", (motorPortName + "/speed_limits/maximum").c_str(), motorConfig.speed_maximum_limit);
      }

      if(privNh.hasParam(motorPortName + "/speed_limits/minimum"))
      {
        privNh.getParam(motorPortName + "/speed_limits/minimum", motorConfig.speed_minimum_limit);
        ROS_INFO("Parameter %s is set to %d", (motorPortName + "/speed_limits/minimum").c_str(), motorConfig.speed_minimum_limit);
      }
    }
    
    //check if the set values for the motor port, angle- and speedlimits are valid
    if (!motorConfig.isValid())
    {
      ROS_FATAL("Motor configuration of motor \"%s\" is invalid.", motorPortNames[motorIdx].c_str());
      return false;
    }
    motor_configurations_[motorIdx] = motorConfig;
  }
  return true;
}

void MotorHandlerNode::MotorHandlerNodeCallback(const abidat_robot_control::MotorControlConstPtr& motor_control_msg, size_t motorIdx)
{
  if(ros::ok  && is_running_)
  {
    last_MotorControll_msg_received = ros::Time::now().toSec(); 
    
    int16_t speed =  motor_control_msg->speed;
    int32_t position = motor_control_msg->angle;

    //if Servo: the transferred angle  will be set
    if(motor_configurations_[motorIdx].is_servo == true && motor_configurations_[motorIdx].is_enabled)
    {
      if(motor_control_msg->angle > motor_configurations_[motorIdx].angle_maximum_limit)
        {
         position = motor_configurations_[motorIdx].angle_maximum_limit;
         ROS_ERROR("The transferred angle must be smaller than %d",  motor_configurations_[motorIdx].angle_maximum_limit);
        }
      if(motor_control_msg->angle < motor_configurations_[motorIdx].angle_minimum_limit)
        {  
          position = motor_configurations_[motorIdx].angle_minimum_limit;
          ROS_ERROR("The transferred angle must be larger than %d",  motor_configurations_[motorIdx].angle_minimum_limit);
        }

      brick_pi_->set_motor_position(motor_configurations_[motorIdx].port_id, position);
    }
    
    // if Motor is in normal mode (wheel mode), the velocity in [deg/sec]m will be set
    if(motor_configurations_[motorIdx].is_servo == false && motor_configurations_[motorIdx].is_enabled)
    {
      if(motor_control_msg->speed > motor_configurations_[motorIdx].speed_maximum_limit)
        {
          speed = motor_configurations_[motorIdx].speed_maximum_limit;
          ROS_ERROR("The transferred velocity must be smaller than %d",  motor_configurations_[motorIdx].speed_maximum_limit);
        }
      if(motor_control_msg->speed < motor_configurations_[motorIdx].speed_minimum_limit)
        {
          speed = motor_configurations_[motorIdx].speed_minimum_limit;
          ROS_ERROR("The transferred velocity must be higher than %d",  motor_configurations_[motorIdx].speed_minimum_limit);
        }
      
      brick_pi_->set_motor_dps( motor_configurations_[motorIdx].port_id, ( motor_configurations_[motorIdx].is_inverse ? speed : (-1) * speed ));
    }
  }

}

bool MotorHandlerNode::initBrickPi()
{
  // Make sure that the BrickPi3 is communicating and that the firmware is compatible with the drivers.
  // Note: Library will return 0 if no errors detected. (#define ERROR_NONE 0)
  if(brick_pi_->detect() != 0)
  {
    ROS_ERROR("MotorHandlerNode: BrickPi is not connected succesfully.");
    return false;
  } 

  else  
    {
      ROS_INFO("MotorHandlerNode: BrickPi is connected succesfully");
      // the buffer variables are needed to 

     for(auto& motor : motor_configurations_)
     {
       // skip configuration if motor isn't enabled
        if(!motor.is_enabled)
        continue;
      
        if (brick_pi_->reset_motor_encoder(motor.port_id) == 1) // int error = 1
          {
            ROS_ERROR("MotorHandlerNode: error occurred during initialization of motor %d.", motor.port);
            return false;
          }
        else
          { 
            ROS_INFO("MotorHandlerNode: encoders for port: %d are resetted to zero.", motor.port);  
          }
        }                     
      }

     
  //Start the thread that publishes the state
  state_thread_ = std::thread(&MotorHandlerNode::readBrickPiValues, this);
  ROS_INFO("MotorHandlerNode: BrickPi state publisher thread was started succesfully");
  return true;
}

void MotorHandlerNode::resetBrickPiValues()
{
  // Reset everything so there are no run-away motors
  //std::lock_guard<std::mutex>guard(mutex_brick_access_);
  brick_pi_->reset_all();
}

void MotorHandlerNode::readBrickPiValues(void)
{
  //std::lock_guard<std::mutex> guard(mutex_brick_access_);
  while(is_running_ && ros::ok())
  {
    //Publish Motor States
    for(std::size_t motorIdx = 0; motorIdx < static_cast<std::size_t>(MotorPort::COUNT); ++motorIdx)
    {
      if(motor_configurations_[motorIdx].is_enabled == true) //maybe instead of  check is.enable, check if the publisher was started 
      {
        abidat_robot_control::MotorState motor_state_msg;
        std_msgs::Header header_;

        header_.seq = 0;
        header_.stamp = ros::Time::now();
        header_.frame_id = "base_link_frame_id";
        motor_state_msg.header = header_;

        brick_pi_->get_motor_status(motor_configurations_[motorIdx].port_id, motor_states_[motorIdx].state, motor_states_[motorIdx].power, motor_states_[motorIdx].position, motor_states_[motorIdx].deg_per_sec);
        
        motor_state_msg.state     = motor_states_[motorIdx].state;
        motor_state_msg.power     = motor_states_[motorIdx].power;
        motor_state_msg.position  = motor_states_[motorIdx].position;
        motor_state_msg.dps       = motor_states_[motorIdx].deg_per_sec; 
        
        pub_motor_states_[motorIdx].publish(motor_state_msg);
        
      }
    }

    //Publish Device info 
    abidat_robot_control::DeviceInfo device_info_msg;
    std_msgs::Header header_;

    header_.seq = 0;
    header_.stamp = ros::Time::now();
    header_.frame_id = "base_link_frame_id";
    
    char info_[33]; // Room for the 32-character serial number string plus the NULL terminator.

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

    pub_device_info.publish(device_info_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void MotorHandlerNode::TimeOutSafety(void)
{
  while (is_running_ == true)
  {
    for(std::size_t motorIdx = 0; motorIdx < static_cast<std::size_t>(MotorPort::COUNT); ++motorIdx)
      {
        safety_mutex_.lock();

        //Stops the robot (set speed to Zero ), if the last receiving massages is older than 0.3s
        if( (ros::Time::now().toSec() - last_MotorControll_msg_received > watchdog_timeout) && motor_configurations_[motorIdx].is_enabled &&  (motor_configurations_[motorIdx].is_servo == false))  //TODO: timeout
        {
          brick_pi_->set_motor_dps(motor_configurations_[motorIdx].port_id, 0);
        }
        
        //checks if the BrickPi can connect to the Motor (check if the connection is still working), if not it gives a warning that the motor isn't connect
        if((motor_states_[motorIdx].state == 2 )&&(motor_configurations_[motorIdx].is_enabled == true))
        {
          ROS_ERROR("The connection to motor %d is lost!", motorIdx);
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
  ros::init(argc, argv, "MotorHandlerNode");
  ros::NodeHandle nh;
  ros::NodeHandle privNh("~");

  abidat::robot::control::MotorHandlerNode motorHandle;
  motorHandle.initialize(privNh, nh);

  ros::spin(); // starts a loop 
  

  return 0;
}

