/**
 * \brief      This ros node is supposed to test if certain published twist messages are subscribed correctly 
 * \author     Egzone Ademi (e.ademi@abidat.de)
 */

#include <gtest/gtest.h>
#include <nodes/forward_kinematic_node.h>
#include <ros/ros.h>
#include "forward_kinematic_calculation.h"

namespace abidat {

namespace robot {

namespace control {


/**
 * \brief Class that contains all necessary attributes to publish and subscribe a twist message 
 */
class TestForwardKinematic : public ::testing::Test
{
public:
  void SetUp() 
  { 
    // initialize publisher and subscriber
    for(std::size_t i = 0; i < forward_kinematics_.getNumMotors(); ++i)
      pub_motor_control_[i] = nh_.advertise<abidat_robot_control::MotorControl>("/officerobot/motor_control_" + i, 1);

    pub_twist_msg_ = nh_.advertise<geometry_msgs::Twist>("/officerobot/cmd_vel", 1);
    velocity_subscriber_ = nh_.subscribe<geometry_msgs::Twist>("/officerobot/cmd_vel", 1, &TestForwardKinematic::callBackTwist, this);        
  }
  
  // initialize twist messages for linear and angular movement
  void publishTwistMsg()
  {
    
    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.angular.z = 0;
    
    ROS_INFO("Publish twist message x = %s, y = %s, z = %s", std::to_string(twist_msg.linear.x).c_str(),
                                                             std::to_string(twist_msg.linear.y).c_str(),
                                                             std::to_string(twist_msg.angular.z).c_str());
    pub_twist_msg_.publish(twist_msg);
  }
  
  // check if the received message is correct
  bool correctTwistMsgReceived() const { return correct_twist_msg_received_ ;}

  /**
  * \brief Receives twist messages for the given time by spinning the nodehandle every 100 ms.
  *
  * \param runTime This function receives twist msgs message the given time. Nodehanlde is spinned every 100 ms.
  */
  void spin(const double run_time)
  {
    const int           number_of_iterations = static_cast<int>(run_time / 0.1);  // 1 sec
    const ros::Duration sleep_time(0.1);

    // receive for one second
    for (int i = 0; i < number_of_iterations; ++i)
    {
      ros::spinOnce();
      sleep_time.sleep();
    }
  }

private:
  void callBackTwist(const geometry_msgs::TwistConstPtr& twist_msg)
  {
    ROS_INFO("Callback Function");

    if (twist_msg->linear.x == 0 && twist_msg->linear.y == 0 && twist_msg->angular.z == 0)
    {
      correct_twist_msg_received_ = true;
    }
    
    //get total movement from forward kinematics
    std::vector<double> velocity;
  
    velocity = forward_kinematics_.calculateForwardKinematics(*twist_msg);

    abidat_robot_control::MotorControl motor_control_msg;

    motor_control_msg.header.stamp = ros::Time::now();
    motor_control_msg.header.frame_id = "base_link";

    for(std::size_t i = 0; i < velocity.size(); i++)
    {
      motor_control_msg.speed = static_cast<std::int16_t>(convertRADToRPM(velocity[i]));
      pub_motor_control_[i].publish(motor_control_msg);
    }
  }

  // create ForwardKinematics object with wheel distance and wheel diameter
  ForwardKinematics forward_kinematics_{0.2, 0.056};
  
  ros::NodeHandle nh_;
  std::array<ros::Publisher, 4> pub_motor_control_;
  ros::Publisher pub_twist_;
  ros::Publisher pub_twist_msg_;
  ros::Subscriber velocity_subscriber_;

  bool correct_twist_msg_received_ = false;
};

// Publish twist messages and check if the correct messages are received 
TEST_F(TestForwardKinematic, MessageReceived)
{
  for(int i = 0; i < 10; i ++)
{  
  this->publishTwistMsg();
  this->spin(0.2);
}
  ASSERT_TRUE(this->correctTwistMsgReceived());
}

} //end namespace

} //end namespace

} //end namespace


int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_forward_kinematic");
  return RUN_ALL_TESTS();
}