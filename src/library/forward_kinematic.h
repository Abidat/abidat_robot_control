/**
 * \brief      This class is used to calculate Forward Kinematics
 *  
 * \author     Egzone Ademi (e.ademi@abidat.de)
 */
#pragma once


//Custom OfficeRobot State Messages
#include <officerobot/MotorControl.h>

//ROS
#include <ros/ros.h>

//ROS Messages
#include <geometry_msgs/Twist.h>

//Thread and thread management
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath> 
#include <array>
#include <vector>


namespace officerobot
{
  /**
   * \brief Interface with abstract methods for forward kinematics calculation  
   */

  class ForwardKinematicsCalculation
  {
  public:
    virtual std::size_t getNumMotors() const = 0;
    virtual std::vector<double> calculateForwardKinematics(const geometry_msgs::Twist& twist_msg) = 0;
  };

  /**
   *\brief Class for the Omniwheel Robot "Marvin" 
   */
  class OfficeRobotForwardKinematics : public ForwardKinematicsCalculation
  {
    public:
      /**
       * \brief Construct a new Office Robot Forward Kinematics object
       * \param distance_wheels distancse between two wheels
       * \param wheel_diamter diameter of each wheel 
       */
      OfficeRobotForwardKinematics(const double distance_wheels, const double wheel_diameter)
        : distance_wheels_(distance_wheels),
          wheel_diameter_(wheel_diameter)
      {

      }

      /**
       * \brief Method that calculates the forward kinematics
       * \param twist_msg the input messages for linear and angular velocity 
       * \return vector that includes the velocity 
       */
      std::vector<double> calculateForwardKinematics(const geometry_msgs::Twist& twist_msg)
      { 
        std::vector<double> velocity;
        // calculate linear component of movement
        double w_linear[Motors::COUNT_MOTORS] = { 0.0 };
         
        const double v_x = twist_msg.linear.x;
        const double v_y = twist_msg.linear.y;
        const double wheel_radius = wheel_diameter_ / 2.0;
        
        w_linear[Motors::MOTOR_A] = ((v_x + v_y) * 4.0) / (-2.8 * wheel_radius);
        w_linear[Motors::MOTOR_B] = ((v_x - v_y) * 4.0) / (-2.8 * wheel_radius);
        w_linear[Motors::MOTOR_C] = -w_linear[Motors::MOTOR_A];
        w_linear[Motors::MOTOR_D] = -w_linear[Motors::MOTOR_B];
    
        // calculate angular component of movement
        const double factor_rot = distance_wheels_ / wheel_diameter_;
        const double w_angular = twist_msg.angular.z * factor_rot;  // Todo check if a minus needs to be insert: -twist_msg.angular.z * factor_rot
      
        // calculate velocity
        double w[Motors::COUNT_MOTORS] = { 0.0 };
    
        for (std::size_t i = 0; i < Motors::COUNT_MOTORS; ++i)
        {
          w[i] = w_linear[i] + w_angular;
          velocity.push_back(w[i]);
        }       
            
        return velocity; 
      }
    
      /**
       * \brief Method to get the number of motors   
       */ 
      std::size_t getNumMotors() const final { return static_cast<std::size_t>(Motors::COUNT_MOTORS); }

      /**
       * \brief Destroy the Office Robot Forward Kinematics object
       */
      ~OfficeRobotForwardKinematics() = default;

    private:
      struct Motors {
        enum {
          MOTOR_A = 0,
          MOTOR_B,
          MOTOR_C,
          MOTOR_D,
          COUNT_MOTORS,
        };
      };
      
      double distance_wheels_; 
      double wheel_diameter_; 

  };
}//end namespace officerobot
#endif
