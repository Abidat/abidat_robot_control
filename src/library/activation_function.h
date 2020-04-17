/**
 * The Activation Function transforms the input value using a function y = ...
 * 
 * \author      Claudia Bina  (c.bina@abidat.de)
 * 
 * \copyright   Abidat GmbH
 */

#include <ros/ros.h>

#include <functional>
#include <assert.h>
#include <cmath>


//Alias for the std::function type
using ActivationFunction = std::function<double(const double)>;


/**
 * \brief Takes the value as input and returns the same value
 * 
 * \param value the input number of the function
 */ 
double linearActivation(const double value)
{
  assert(value >= -1.0 && value <= 1.0);
  return value;
}


/**
 * \brief Takes a value as input and transforms it into output f(value) = value^x ....
 * 
 * \param value the input number of the function
 */ 
double expoActivation(const double value)
{
  //Some functions that can be tested on the robot
  //(x*x) / (2-x)
  //(x*x) / (3-x)
  //std::pow(value, 3);

  return std::pow(value, 2) / (2-value);
}


ActivationFunction createActivationFunction(const std::string& name)
{
  if (name == "linear")
    return linearActivation;
  else if (name == "exponential")
    return expoActivation;
  else
    ROS_ERROR("createActivationFunction(): the specified function does not exist.");
}