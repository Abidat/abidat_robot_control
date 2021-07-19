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

namespace abidat {

namespace robot {

namespace control {

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
  assert(value >= -1.0 && value <= 1.0);
  return value * value * (value < 0.0 ? -1.0 : 1.0);
}


double polyActivation(const double value)
{
  assert(value >= -1.0 && value <= 1.0);
  return (value * value * value + value) / 2.0;
}

ActivationFunction createActivationFunction(const std::string& name)
{
  if (name == "linear") {
    return linearActivation;
  }
  else if (name == "exponential") {
    return expoActivation;
  }
  else if (name == "polynomial") {
    return polyActivation;
  }
  else {
    ROS_ERROR("createActivationFunction(): the specified function does not exist.");
  }

  // return empty function
  // @todo it is better to throw an exception here or return std::optional
  return { };
}

} //end namespace control

} //end namespace robot

} //end namespace abidat
