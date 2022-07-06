/**
 *   This class maps the indices of the controller to InputIndices and MaxVelocities structures.
 *   It also computes velocity based on axes values and activation function
 * 
 * \author      Claudia Bina  (c.bina@abidat.de)
 * \author      Christian Wendt (c.wendt@abidat.de)
 * 
 * \copyright   Abidat GmbH
 */

#include <optional>

#include <geometry_msgs/msg/twist.hpp>
#include <sstream>
#include <stdexcept>

#include "abidat_robot_control/base/input_mapping.h"

namespace abidat {

namespace robot {

namespace control {

/**
 * \brief Checks if a given indices structure has been only initialized.
 *        If it has been properly intialized, then its members should have negative values
 * 
 * \param indices the structure that needs to be checked
 */ 
bool InputMapping::isInitializedIndices(const InputIndicies &indices) const
{
    //the three members of indices should be initialised with negative values
    if (indices.idx_velocity_angular < 0
        &&
        indices.idx_velocity_x < 0
        &&
        indices.idx_velocity_y < 0)
    {
      return true;
    }

    return false;
}


/**
 * \brief Checks if a given MaxVelocities structure has been only initialized.
 *        If it has been properly intialized, then its members should have negative values
 * 
 * \param max the structure that needs to be checked
 */
bool InputMapping::isInitializedVelocities(const MaxVelocities& max) const
{
    //maximum velocities should be initialised with a negative value
    if(max.max_angular < 0
       &&
       max.max_linear < 0) 
    {
        return true;
    }

    return false;
}

/**
 * \brief Takes the axes values from the axes vector, uses activation function and computes velocity. Returns twist message.
 * 
 * \param axes the axes vector of the received joy message 
 * \param buttons the buttons vector of the received joy message
 *        
 */ 
std::optional<geometry_msgs::msg::Twist> InputMapping::computeVelocity(const std::vector<float>& axes,
                                                                  const std::vector<std::int32_t>& buttons) const
{
    //indices should be smaller than axes size
    if(input_indicies_.idx_velocity_x >= axes.size()
       ||
       input_indicies_.idx_velocity_y >= axes.size() 
       ||
       input_indicies_.idx_velocity_angular >= axes.size())
    {
        std::stringstream error_msg;
        error_msg << "InputMapping::computeVelocity(): indices out range\n";
        error_msg << "Indices should be < " << axes.size() << " and they are:" << '\n';
        error_msg << "idx_velocity_x       = " << input_indicies_.idx_velocity_x << '\n';
        error_msg << "idx_velocity_y       = " << input_indicies_.idx_velocity_y << '\n';
        error_msg << "idx_velocity_angular = " << input_indicies_.idx_velocity_angular << '\n';

        throw std::runtime_error(error_msg.str());
    }

    //indices should have values greater than 0, so they need to be also set, not only initialized
    if( isInitializedIndices(input_indicies_)
        ||
        isInitializedVelocities(max_velocities_) )
    {
        throw std::runtime_error("InputMapping::computeVelocity(): input_indices or max_velocities not set");
    };

    geometry_msgs::msg::Twist twistMsg;

    // 1. get axes values from axes vector using indices
    float xVelocityValue;
    float yVelocityValue;
    float angularVelocityValue;

    xVelocityValue       = axes[input_indicies_.idx_velocity_x ];
    yVelocityValue       = axes[input_indicies_.idx_velocity_y]; 
    angularVelocityValue = axes[input_indicies_.idx_velocity_angular];

    // 2. adapt axes values using activation function
    xVelocityValue       = activation_function_(xVelocityValue);
    yVelocityValue       = activation_function_(yVelocityValue);
    angularVelocityValue = activation_function_(angularVelocityValue);

    // 3. convert adapted axes values to velocity (multiply by max_linear)
    twistMsg.linear.x  = xVelocityValue       * max_velocities_.max_linear;
    twistMsg.linear.y  = yVelocityValue       * max_velocities_.max_linear;
    twistMsg.angular.z = angularVelocityValue * max_velocities_.max_angular;

    return twistMsg;
}

std::optional<geometry_msgs::msg::Twist> InputMapping::computeVelocity(const int& key)
{
    geometry_msgs::msg::Twist twistMsg;
    
    switch (key)
    {
        case InputKeys::KEY_W:  // forwards
            twistMsg.linear.x = linear_keyboard_speed_;
            twistMsg.linear.y = 0;
            twistMsg.angular.z = 0;
            break;
        case InputKeys::KEY_A:  // leftwards
            twistMsg.linear.x = 0;
            twistMsg.linear.y = linear_keyboard_speed_;
            twistMsg.angular.z = 0;
            break;
        case InputKeys::KEY_S:  // backwards
            twistMsg.linear.x = -linear_keyboard_speed_;
            twistMsg.linear.y = 0;
            twistMsg.angular.z = 0;
            break;
        case InputKeys::KEY_D:  // rightwards
            twistMsg.linear.x = 0;
            twistMsg.linear.y = -linear_keyboard_speed_;
            twistMsg.angular.z = 0;
            break;
        case InputKeys::KEY_Q: // turn to the left
            twistMsg.linear.x = 0;
            twistMsg.linear.y = 0;
            twistMsg.angular.z = angular_keyboard_speed_;    
            break;
        case InputKeys::KEY_E: // turn to the right
            twistMsg.linear.x = 0;
            twistMsg.linear.y = 0;
            twistMsg.angular.z = -angular_keyboard_speed_;  
            break;
        default:
            throw std::runtime_error("Illegal keyboard input");
            break;
    }

    return twistMsg;
}

} //end namespace control

} //end namespace robot

} //end namespace abidat