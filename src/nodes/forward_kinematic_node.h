#include "forward_kinematic.h"

//ROS
#include <ros/ros.h>

//ROS Messages
#include <geometry_msgs/Twist.h>
#include <std_msgs/Header.h>

namespace abidat {

namespace robot {

namespace control {

class ForwardKinematicNode 
{

public:
  // default constructor
  ForwardKinematicNode() = default;

  // default destructor
  ~ForwardKinematicNode() = default;

  /**
   * \brief Method that initializes the motor control publisher and the velocity subscriber
   * \param node the ros Nodehandler 
   * \param veloctiy_subscriber subscriber initialized with twist messages
   */ 
  bool initialize(ros::NodeHandle& private_nh, ros::NodeHandle& node);
  
  private:
    /**
     * \brief Method that will be called when a new twist message arrives
     * \param twist_msg the input messages to be received 
     */
    void callback(const geometry_msgs::TwistConstPtr& twist_msg);

    /**
     * \brief Method that reads the parameters for the robot from the param server. 
     * Parameters distance_wheels and wheel_diameter are needed to intialize the forward kinematic class
     * \return true if successfull.
     */ 
    // bool readParams();

    double distance_wheels_;
    double wheel_diameter_;

    ros::Subscriber velocity_subscriber_; //> subscriber for velocity
    std::array<ros::Publisher,4> pub_motor_control_; //> publisher for the motor control for each existing motor 
    std::shared_ptr<ForwardKinematics> forward_kinematics_; //> shared pointer declaration of type ForwardKinematics
  
}; 

} //end namespace control

} //end namespace robot

} //end namespace abidat