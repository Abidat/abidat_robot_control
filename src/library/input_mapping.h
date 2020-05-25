/**
 *   This class maps the indices of the controller to InputIndices and MaxVelocities structures.
 *   It also computes velocity based on axes values and activation function
 * 
 * \author      Claudia Bina  (c.bina@abidat.de)
 * 
 * \copyright   Abidat GmbH
 */

#include <geometry_msgs/Twist.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <math.h>

#include "activation_function.h"
#include "get_keyboard_input.h"

/**
 * \brief Indices for the axes vector of the joy message. Can be set in the tele_op.yaml parameter file.
*/
struct InputIndicies
{
  int32_t idx_velocity_x;
  int32_t idx_velocity_y;
  int32_t idx_velocity_angular;
};

/**
 * \brief The maximum values for linear and angular velocities. Can be set in the tele_op.yaml parameter file.
*/
struct MaxVelocities
{
  double max_linear;
  double max_angular;
};

struct InputKeys 
{
	enum 
	{
    KEY_Q = 113,
		KEY_W = 119,
		KEY_E = 101,
		KEY_A = 97,
		KEY_S = 115,
    KEY_D = 100
	};
};


class InputMapping
{
public:
  InputMapping() = default;

  inline void setIndices(const InputIndicies& indices)                   { input_indicies_        = indices; }
  inline void setMaxVelocities(const MaxVelocities& max)                 { max_velocities_        = max; }
  inline void setActivationFunction(const ActivationFunction function)   { activation_function_   = function; }
  inline void setLinearKeyboardSpeed(const float& linear_keyboard_speed) { linear_keyboard_speed_  = linear_keyboard_speed; }
  inline void setAngularKeyboardSpeed(const float& angular_keyboard_speed){ angular_keyboard_speed_ = angular_keyboard_speed; }


  inline InputIndicies getIndices()        {return input_indicies_;}
  inline MaxVelocities getVelocities()     {return max_velocities_;}

/**
 * \brief Takes the axes values from the axes vector, uses activation function and computes velocity. Returns twist message.
 * 
 * \param axes the axes vector of the received joy message 
 * \param buttons the buttons vector of the received joy message
 *        
 */ 
  std::optional<geometry_msgs::Twist> computeVelocity(const std::vector<float>& axes, 
                                                      const std::vector<std::int32_t>& buttons) const;

/**
 * \brief Takes the key input from the getKey function and remapps it into a twist message.
 * 
 * \param key the integer value of the pressed key recieved by GetKeyboardInput::getKey.
 */ 
  std::optional<geometry_msgs::Twist> computeVelocity(const int& key);

private:

  /**
   * \brief Checks if a given indices structure has been only initialized.
   *        If it has been properly intialized, then its members should have negative values
   * 
   * \param indices the structure that needs to be checked
   */ 
  bool isInitializedIndices(const InputIndicies &indices) const;


  /**
   * \brief Takes the axes values from the axes vector, uses activation function and computes velocity. Returns twist message.
   * 
   * \param axes the axes vector of the received joy message 
   * \param buttons the buttons vector of the received joy message
   *        
   */ 
  bool isInitializedVelocities(const MaxVelocities& max) const;
  
  ActivationFunction activation_function_ { linearActivation };
  InputIndicies input_indicies_{-1, -1, -1};
  MaxVelocities max_velocities_{-1, -1};  
  float linear_keyboard_speed_;
  float angular_keyboard_speed_;
};