#include <gtest/gtest.h>
#include <input_mapping.h>
#include <ros/console.h>

namespace abidat {

namespace robot {

namespace control {
/**
 * \brief Tests if an object of type InputMapping is correctly instantiated.
 *        Tests if error is correctly thrown when trying to use two empty vectors for computing the velocity
 */ 
TEST(InputMapper, Instantiate)
{
  InputMapping mapper;

  // test default behaviour
  std::vector<float> axes;
  std::vector<std::int32_t> buttons;

  ASSERT_FALSE(mapper.computeVelocity(axes,buttons));
}


/**
 * \brief Tests if, at instantiation, the correct default values are set for the input_indices_ member
 */ 
TEST(InputMapper, DefaultIndicesValues)
{
  InputMapping mapper;
  InputIndicies inputInd_mapper;

  inputInd_mapper = mapper.getIndices();

  ASSERT_EQ(inputInd_mapper.idx_velocity_x       , -1);
  ASSERT_EQ(inputInd_mapper.idx_velocity_y       , -1);
  ASSERT_EQ(inputInd_mapper.idx_velocity_angular , -1);
}



/**
 * \brief Tests if, at instantiation, the correct default values are set for the max_velocities_ member
 */ 
TEST(InputMapper, DefaultVelocitiesValues)
{
  InputMapping mapper;
  MaxVelocities maxVel_mapper;

  maxVel_mapper = mapper.getVelocities();

  ASSERT_EQ(maxVel_mapper.max_angular , -1);
  ASSERT_EQ(maxVel_mapper.max_linear  , -1);
}



/**
 * \brief Dummy linear activation function user to test the compute velocity method.
 */
double dummyActivation(const double value) { return value; }



/**
 * \brief Tests if compute velocity function is working properly when a certain activation
 *        function is set.
 */ 
TEST(InputMapper, ComputeVelocityLinearActivation)
{
  InputMapping mapper;
  std::vector<float> axes = { 1.0, 0.5, -1.0 };
  std::vector<std::int32_t> buttons;

  mapper.setActivationFunction(dummyActivation);
  mapper.setMaxVelocities({1,1});
  mapper.setIndices({ 0, 1, 2 });

  auto twistMessage = mapper.computeVelocity(axes, buttons);

  ASSERT_TRUE(twistMessage);

  EXPECT_EQ(twistMessage->linear.x, 1.0);
  EXPECT_EQ(twistMessage->linear.y, 0.5);
  EXPECT_EQ(twistMessage->linear.z, 0);

  EXPECT_EQ(twistMessage->angular.x, 0);
  EXPECT_EQ(twistMessage->angular.y, 0);
  EXPECT_EQ(twistMessage->angular.z, -1.0);
}

} //end namespace control

} // end namespace robot

} // end namespace abidat

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}