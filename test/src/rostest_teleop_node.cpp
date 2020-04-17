/**
 * \brief        This file is a node used to test the officerobot_tele_op node.
 *
 * \author       Claudia Bina <c.bina@abidat.de>
 *
 * \copyright    Copyright (C) Abidat GmbH
 */

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <stdio.h>


//Default data for the joy message
const std::vector<float>        axesDefault    = {1.0, 0.5, 0, 1.0, 0, 0};
const std::vector<std::int32_t> buttonsDefault = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const std::vector<float>        axesNegativeTest = {1.0, 0}; 


/**
 * \brief Returns the default twist message expected when the default joy message is sent
 */
geometry_msgs::Twist ExpectedTwistWhenLinearActivationUsed()
{
  geometry_msgs::Twist expected_twist;
  expected_twist.linear.x  = 5.0;
  expected_twist.linear.y  = 10.0;
  expected_twist.angular.z = 10.0;

  return expected_twist;
}


class TestTeleOp: public testing::Test
{
  public:
    TestTeleOp()  = default;
    ~TestTeleOp() = default;
    

    /**
     * \brief Setup of the class. Currently it sets the publisher and subscriber
     */ 
    void SetUp() override 
    {
      publisher_  = nh_.advertise<sensor_msgs::Joy>("/joy", 1);
      subscriber_ = nh_.subscribe("officerobot/cmd_vel", 1, &TestTeleOp::callbackTeleOp, this);
    }


    /**
     * \brief After each test set the boolean flags to their initial values
     */ 
    void TearDown() override
    {
      twist_msg_received       = false;
      valid_twist_msg_received = true;
    }


  protected:

    /**
     * \brief Spin the test node
     * 
     * \param run_time represents the runtime in seconds
     */ 
    void spin(const double run_time)
    {
      const int           num_of_iterations = static_cast<int>(run_time / 0.1);
      const ros::Duration sleep_time(0.1);

      for (int i = 0; i < num_of_iterations; ++i)
      {
        ros::spinOnce();
        sleep_time.sleep();
      }
    }

    
    /**
     * \brief Construct and publish a joy message. The same joy message is published everytime. 
     * 
     * \param buttons the buttons array of the joy message. It is defaulted to buttonsDefault defined above.
     * \param axes    the axes array of the joy message. It is defaulted to axesDefault defined above.
     */ 
    void PublishJoyMessage(const std::vector<std::int32_t>& buttons = buttonsDefault, const std::vector<float>& axes = axesDefault)
    {
      sensor_msgs::Joy msg;
      msg.header.stamp    = ros::Time::now();
      msg.axes            = axes;
      msg.buttons         = buttons;

      publisher_.publish(msg);
    }


    /**
     * \brief Return the status of the boolean flag indicating if a twist message has been received.
     */ 
    bool TwistMsgReceived()      {return twist_msg_received; }


    /**
     * \brief Return the status of the boolean flag indicating if a valid twist message has been received.
     */ 
    bool ValidTwistMsgReceived() {return valid_twist_msg_received; }


  private:
    /**
     * \brief Callback of the node 
     */ 
    void callbackTeleOp (const geometry_msgs::Twist& msg )
    {
      twist_msg_received = true;

      //testing just msg.linear.x, msg.linear.y and msg.angular.z 
      //because the others are not used, so are always 0
      valid_twist_msg_received &= (expected_msg.linear.x == msg.linear.x
                                   &&
                                   expected_msg.linear.y == msg.linear.y
                                   &&
                                   expected_msg.angular.z == msg.angular.z);
    }

    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Publisher  publisher_;
    
    //boolean flags used for tests
    bool twist_msg_received       = false;

    //at the beginning we suppose that the received message is valid
    bool valid_twist_msg_received = true;

    //data member saving the data of the expected received message
    const geometry_msgs::Twist expected_msg = ExpectedTwistWhenLinearActivationUsed();
};


/**
 * \brief Publish the same joy message multiple times and test if the same 
 *        twist message is received everytime 
 */ 
TEST_F(TestTeleOp, PublishJoyMsg)
{
  for (std::size_t i = 0; i < 50; ++i)
  {
    this->spin(0.1);
    this->PublishJoyMessage();
  }

  ASSERT_TRUE(this->TwistMsgReceived());
  ASSERT_TRUE(this->ValidTwistMsgReceived());
}


/** 
 * \brief Check the TearDown method 
 */ 
TEST_F(TestTeleOp, TwistMsgReceived)
{
  this->spin(0.1);

  ASSERT_FALSE(this->TwistMsgReceived() );
}


/** 
 * \brief Negative test.
 *        Publish a not valid joy message and test that no twist message is published back.
 */
TEST_F(TestTeleOp, PublishNotValidJoyMsg)
{
  this->spin(0.1);

  //The axes vector used is of size 2
  //This will generate "InputMapping::computeVelocity(): indices out range" warning which will
  //    lead to "OfficeRobotTeleOp::joyCallback(): Couldn t publish. twistMsg not valid." error.
  //So no twist message should be published back by the TeleOp node.
  this->PublishJoyMessage(buttonsDefault, axesNegativeTest);

  ASSERT_FALSE(this->TwistMsgReceived() );
}


int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_tele_op");

  return RUN_ALL_TESTS();
}