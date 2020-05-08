/**
 * \brief      This file includes calculations needed for the forwards kinematic
 *  
 * \author     Egzone Ademi (e.ademi@abidat.de)
 * \maintainer Egzone Ademi (e.ademi@abidat.de)
 */

#include <cmath>

namespace abidat {

namespace robot {

namespace control {

/**
 * \brief Function that converts rad/s to rpm
 * \param movement the total movement from the forwards kinematics
 * \return the movement converted to rpm
 */
double convertRADToRPM(double movement)
{
    movement = movement * 60.0 / M_PI * 2.0;
    
    return movement;
}

} //end namespace control

} //end namespace robot

} //end namespace abidat