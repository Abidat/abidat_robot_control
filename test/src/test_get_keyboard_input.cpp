#include <gtest/gtest.h>
#include <get_keyboard_input.h>

class KeyboardFixture : public ::testing::Test
{
public:
    void SetUp() override
    {

    }

    void TearDown() override
    {

    }

protected:
    void getKeyboardInputCallback(const char key)
    {
        switch (key) {
            case 'a':
                counter_a++;
                break;

            case 'b':
                counter_b++;
                break;

            case 'c':
                counter_c++;
                break;

            case 'd':
                counter_d++;
                break;

            default:
                ASSERT_TRUE(false) << "unexpected key received: \"" << key << "\"";
                break;
        }
    }

    GetKeyboardInput device_under_test_{std::bind(&KeyboardFixture::getKeyboardInputCallback, this, std::placeholders::_1)};
    std::atomic<unsigned int> counter_a{0};
    std::atomic<unsigned int> counter_b{0};
    std::atomic<unsigned int> counter_c{0};
    std::atomic<unsigned int> counter_d{0};
};

/**
 * \brief 
 */ 

TEST(GetKeyboardInput, Instantiate)
{
    GetKeyboardInput input([](const char a){});
}

TEST(GetKeyboardInput, StartAndStop)
{
    GetKeyboardInput input([](const char a){});
    EXPECT_FALSE(input.isRunning());
    
    input.start();
    EXPECT_TRUE(input.isRunning());

    input.stop();
    EXPECT_FALSE(input.isRunning());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}