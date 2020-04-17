/**
 * \brief      This class is used to drive the BrickPi motor ports
 *  
 * \author     Mergim Halimi (m.halimi@abidat.de)
 * \maintainer Christian Merkl (c.merkl@abidat.de)
 */
#pragma once

//ROS
#include <ros/ros.h>

//ROS Messages
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>

//Custom OfficeRobot State Messages
// \todo: actually it can be removed
#include <officerobot/MotorState.h>
#include <officerobot/DeviceInfo.h>
#include <officerobot/MotorControl.h>

//Thread and thread management
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

//BrickPi
#include "brick_pi/BrickPi3.h"

/**
 * @brief The namespace used for OfficeRobot
 * 
 */
namespace officerobot
{

/**
 * @brief This class handles a brick pie motor using the brick pie library. This class is a ROS node.
 * 
 */
class MotorHandlerNode 
{
  public:
  /**
   * @brief This enum contains all available ports on the BrickPi
   * 
   */
    enum class MotorPort : uint8_t
    {
      PORTA = 0,
      PORTB,
      PORTC,
      PORTD,
      COUNT,
      UNKOWN,
    };

    /**
     * \brief Contains the necessary motor configuration.
     */
    struct MotorConfiguration
    {
     // Variables for storing if the motor is enabled 
      bool is_enabled = false;  

      // Variables for storing if the motor is  working in inverse mode
      bool is_inverse = false;

      // Variables for storing if the motor is running in servo mode
      bool is_servo = false;

      MotorPort port = MotorPort::UNKOWN;
      // Variables for storing the specified port in HEX
      std::uint8_t port_id = 0;

      // Variables for storing the middle position used for servo mode
      std::int32_t middle_position = 0;


      //max permitted angle, the steering could be set [pos]
      std::int32_t angle_maximum_limit = 0;
      std::int32_t angle_minimum_limit = 0;

      //max permitted velocity [dps]
      std::int32_t speed_maximum_limit = 0;
      std::int32_t speed_minimum_limit = 0;

          /**
       * \brief Checks if this configuration is valid. The struct must be invalid after construction.
       * 
       * \return true if this configuration is valid.
       */

      bool isValid(void) const
      {
        return port != MotorPort::UNKOWN
               &&
               port != MotorPort::COUNT
               &&
               port_id != 0
               &&
               (angle_maximum_limit >= angle_minimum_limit)
               &&
               (speed_maximum_limit >= speed_minimum_limit);
      }


    };

    /**
     * \brief Contains the current motor state.
     */
    struct MotorState
    {
      // Variables for reading motor state bits
      std::uint8_t state = 0; // TODO: maybe remove it from the struct

      // Variables for reading motor powers
      std::int8_t power = 0;

      // Variables for reading motor encoder positions
      std::int32_t position = 0;

      // Variables for reading motor speeds (Degrees Per Second)
      std::int16_t deg_per_sec = 0;
    };


    /**
     * @brief Construct a new Office Robot Motor Driver object
     * 
     */
    MotorHandlerNode(void);

    /**
     * @brief Destroy the Office Robot Motor Driver object
     * 
     */
    ~MotorHandlerNode(void);

    /**
     * \brief Initialize this node using the given node handles.
     * 
     * \param privNh necessary private node handle.
     * \param nh necessary public node handle.
     * \return true if initialization was successful.
     */
    bool initialize(ros::NodeHandle& privNh, ros::NodeHandle& nh);

  private:
    /**
     * @brief This is the callback for the Twist message topic
     * 
     * @param motor_control Velocity that is published into the topic
     */
    void MotorHandlerNodeCallback(const officerobot::MotorControlConstPtr& motor_control,size_t motorIdx);
    
    /**
     * @brief This function instantiate the BrickPi and checks if the connection with the harware is succesfull or not.
     * 
     * @return true If initialization is done without error
     * @return false If initialisation failes, this will cause the node stop
     */
    bool initBrickPi(void);

    /**
     * @brief This function is called from the callback, it will get the speed (do a calculation if nessesary)
     *        and run the motors
     * 
     * @param linear_speed Speed in DPS
     */
    void runBrickPiMotor(float linear_speed, MotorState &motor);

    /**
     * @brief This function is called from the callback, it will get the speed (do a calculation if nessesary)
     *        and set the servo motor position for turning
     * 
     * @param angular_speed Value of the position of the servo motor
     */
    void setBrickPiMotorPosition(float angular_speed, MotorState &motor);

    /**
     * @brief Function that reads all the parameters from the specified port
     * 
     * @param motor_ Object refference that stores all the states, including the name and port
     */
    void readBrickPiValues(void);

    /**
     * @brief Function that resets all the values, and makes sure that the motors have no power and encoders state is set to 0
     * 
     */
    void resetBrickPiValues();

    /**
     * @brief Function that reads all the parameters from the launch file
     * 
     * @return true if the read is done succefully
     * @return false if the read has failed, this will also cause the stop of the node
     */
    bool readMotorConfigurations(ros::NodeHandle& privNh);

    /**
     * @brief Function to calibrate the turning of wheels, this function will reset the position to steering_mid_possition
     *        that we can change from the launch file. Inside the function, the limits will change accordint to steering_mid_possition
     * 
     */
    bool calibrateSteering(const MotorConfiguration& motor);



    void TimeOutSafety(void);
    /**
     * @brief Function check if the Subscriber is receiving massages for the MotorControll, 
     * if the officebot_motor_handler doesn't receive a massage(for example in case of network problems) for a period time the Robot stops 
     * 
     */


    // members
    bool is_initialized_ = false;
    double last_MotorControll_msg_received = 0;
    double watchdog_timeout = 0.3; // todo: set this Patameter over launch file. 

    // the size of the following arrays is alltime the maximum possible count, but can contain uninitialized (unused) motors.
    std::array<MotorConfiguration, static_cast<std::size_t>(MotorPort::COUNT)> motor_configurations_;
    std::array<MotorState        , static_cast<std::size_t>(MotorPort::COUNT)> motor_states_;
    std::array<ros::Subscriber   , static_cast<std::size_t>(MotorPort::COUNT)> sub_motor_controlls_;
    std::array<ros::Publisher    , static_cast<std::size_t>(MotorPort::COUNT)> pub_motor_states_;

    ros::Publisher pub_device_info;   // Publisher for device info (was not sure where else I could write it )

    // get state from motors thread
    std::thread state_thread_;
    std::thread safety_thread_;
    std::mutex mutex_brick_access_;
    std::mutex safety_mutex_;
    
    std::atomic<bool> is_running_ { true };

    /**
     * @param[brick_pi_] : BrickPi3 instance 
     * @param[motors_]   : Vector with enabled motors
     */
    std::unique_ptr<BrickPi3> brick_pi_;
  
  };

} //end namespace officerobot

