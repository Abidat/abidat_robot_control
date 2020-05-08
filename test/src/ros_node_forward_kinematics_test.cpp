/**
 * \brief      This ros node is supposed to test if certain published twist messages are subscribed correctly 
 * \author     Egzone Ademi (e.ademi@abidat.de)
 */

#include <gtest/gtest.h>
#include <nodes/officerobot_forward_kinematics.h>
#include <ros/ros.h>
#include "forward_kinematics_calculations.h"


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
    ROS_INFO("Publish twist message");
    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.angular.z = 0;

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

    // check if the subscribed messages are the same as the published ones -> TODO: remove hard coded numbers?
    if (twist_msg->linear.x == 0 && twist_msg->linear.y == 0 && twist_msg->angular.z == 0)
    {
      correct_twist_msg_received_ = true;
    }
    
    //get total movement from forward kinematics
    std::vector<double> total_movement;
  
    total_movement = forward_kinematics_.calculateForwardKinematics(*twist_msg);

    officerobot::MotorControl motor_control_msg;

    motor_control_msg.header.stamp = ros::Time::now();
    motor_control_msg.header.frame_id = "base_link";

    for(std::size_t i = 0; i < total_movement.size(); i++)
    {
      motor_control_msg.speed = static_cast<std::int16_t>(convertRADToRPM(total_movement[i]));
      pub_motor_control_[i].publish(motor_control_msg);
    }
  }

  // create OfficeRobotForwardKinematics object with wheel distance and wheel diameter
  OfficeRobotForwardKinematics forward_kinematics_{0.2, 0.056};
  
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


int main(int argc, char** argv)
{
  //TestForwardKinematic forward_kin;
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "basic_function_test");
  return RUN_ALL_TESTS();
}