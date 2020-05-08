#include <gtest/gtest.h>
#include "nodes/forward_kinematic_node.h"


// Test forward kinematics calculation for "Marvin"

TEST(ForwardKiematics, NoMovement)
{  
   //vector to store value of total movement 
   std::vector<double> total_movement;

   // Set values for linear and angular movement  
   auto twist_msg = std::make_shared<geometry_msgs::Twist>();
   twist_msg->linear.x = 0.0;
   twist_msg->linear.y = 0.0;
   twist_msg->angular.z = 0.0;
  
   // create object of ForwardKinematics to use its FK calculation
   abidat::robot::control::ForwardKinematics forward_kinematics(0.2, 0.056);
   
   total_movement = forward_kinematics.calculateForwardKinematics(*twist_msg);
   
   // check if the movement of every wheel is 0 
   for(std::size_t i = 0; i < total_movement.size(); i++)
   ASSERT_EQ(total_movement.at(i), 0);

}

TEST(ForwardKiematics, LinearXMovement)
{  
   //vector to store value of total movement 
   std::vector<double> total_movement;
   
   // Set values for linear and angular movement  
   auto twist_msg = std::make_shared<geometry_msgs::Twist>();
   twist_msg->linear.x = 1.0;
   twist_msg->linear.y = 0.0;
   twist_msg->angular.z = 0.0;
  
   // create object of ForwardKinematics to use its FK calculation
   abidat::robot::control::ForwardKinematics forward_kinematics(0.2, 0.056);
   
   total_movement = forward_kinematics.calculateForwardKinematics(*twist_msg);
   
   // Check if the movement was calculated correctly, allowing an absolute error of 0.001
   ASSERT_NEAR(total_movement.at(0), -51.02, 0.001);
   ASSERT_NEAR(total_movement.at(1), -51.02, 0.001);
   ASSERT_NEAR(total_movement.at(2),  51.02, 0.001);
   ASSERT_NEAR(total_movement.at(3),  51.02, 0.001);
}

TEST(ForwardKinematics, LinearYMovement)
{
  //vector to store value of total movement 
   std::vector<double> total_movement;
   
   // Set values for linear and angular movement  
   auto twist_msg = std::make_shared<geometry_msgs::Twist>();
   twist_msg->linear.x = 0.0;
   twist_msg->linear.y = 1.0;
   twist_msg->angular.z = 0.0;
  
   // create object of ForwardKinematics to use its FK calculation
   abidat::robot::control::ForwardKinematics forward_kinematics(0.2, 0.056);
   
   total_movement = forward_kinematics.calculateForwardKinematics(*twist_msg);
   
   // Check if the movement was calculated correctly, allowing an absolute error of 0.001
   ASSERT_NEAR(total_movement.at(0), -51.02, 0.001);
   ASSERT_NEAR(total_movement.at(1),  51.02, 0.001);
   ASSERT_NEAR(total_movement.at(2),  51.02, 0.001);
   ASSERT_NEAR(total_movement.at(3), -51.02, 0.001);
}

TEST(ForwardKinematics, RotationAroundZaxis)
{
     //vector to store value of total movement 
   std::vector<double> total_movement;
   
   // Set values for linear and angular movement  
   auto twist_msg = std::make_shared<geometry_msgs::Twist>();
   twist_msg->linear.x = 0.0;
   twist_msg->linear.y = 0.0;
   twist_msg->angular.z = 1.0;
  
   // create object of ForwardKinematics to use its FK calculation
   abidat::robot::control::ForwardKinematics forward_kinematics(0.2, 0.056);
   
   total_movement = forward_kinematics.calculateForwardKinematics(*twist_msg);
   
   // Check if the movement was calculated correctly, allowing an absolute error of 0.001
   ASSERT_NEAR(total_movement.at(0), 3.571, 0.001);
   ASSERT_NEAR(total_movement.at(1), 3.571, 0.001);
   ASSERT_NEAR(total_movement.at(2), 3.571, 0.001);
   ASSERT_NEAR(total_movement.at(3), 3.571, 0.001);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}