#include "teleoperation_node.h"

namespace officerobot {

    /**
     * \brief Reads the parameters from the parameter server and maps them into the input mapper, It also sets the activation function.
     */
    bool TeleoperationNode::readParameters() 
    {
        ros::NodeHandle private_nh("~");
        InputIndicies indices;
        MaxVelocities maxVelocities;
        std::string activation_function;
        float linear_keyboard_speed = 0.5;
        float angular_keyboard_speed = 0.75;

        //getting parameters from the parameter server
        if(private_nh.hasParam("index_linear_speed_x"))
        {
            private_nh.getParam("index_linear_speed_x", indices.idx_velocity_x);
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): idx_velocity_x not found. Using the default value.");
        }


        if(private_nh.hasParam("index_linear_speed_y"))
        {
            private_nh.getParam("index_linear_speed_y", indices.idx_velocity_y);
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): idx_velocity_y not found. Using the default value.");
        }


        if(private_nh.hasParam("index_angular_speed"))
        {
            private_nh.getParam("index_angular_speed", indices.idx_velocity_angular);
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): idx_velocity_angular not found. Using the default value.");
        }


        if(private_nh.hasParam("max_linear_speed"))
        {
            private_nh.getParam("max_linear_speed", maxVelocities.max_linear);
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): max_linear_speed not found. Using the default value.");
        }


        if(private_nh.hasParam("max_angular_speed"))
        {
            private_nh.getParam("max_angular_speed", maxVelocities.max_angular);
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): max_angular_speed not found. Using the default value.");
        }

        if(private_nh.hasParam("activation_function"))
        {
            private_nh.getParam("activation_function", activation_function);
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): activation_function not found. Using the default value.");
        }

        if(private_nh.hasParam("linear_keyboard_speed"))
        {
            private_nh.getParam("linear_keyboard_speed", linear_keyboard_speed);
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): linear_keyboard_speed not found. Using the default value.");
        }

        if(private_nh.hasParam("angular_keyboard_speed"))
        {
            private_nh.getParam("angular_keyboard_speed", angular_keyboard_speed);
            ROS_INFO("TeleoperationNode::readParameters(): angular_keyboard_speed %f.", angular_keyboard_speed);
            std::cout << angular_keyboard_speed;
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): angular_keyboard_speed not found. Using the default value.");
        }

        if(private_nh.hasParam("keyboard_enable"))
        {
            private_nh.getParam("keyboard_enable", keyboard_enable_);
        }
        else
        {
            ROS_INFO("TeleoperationNode::readParameters(): keyboard_enable not found. Using the default value.");
        }
        


        //Setting parameter into the input mapper
        input_mapper_.setIndices(indices);
        input_mapper_.setMaxVelocities(maxVelocities);
        input_mapper_.setActivationFunction(createActivationFunction(activation_function));
        input_mapper_.setLinearKeyboardSpeed(linear_keyboard_speed);
        input_mapper_.setAngularKeyboardSpeed(angular_keyboard_speed);

        return true;
    }


    void TeleoperationNode::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
        auto msg = input_mapper_.computeVelocity(joy->axes, joy->buttons);
        if (msg) {
            pub_twist_.publish(*msg);
        }
        else {
            // print error
            ROS_ERROR("TeleoperationNode::joyCallback(): Couldn t publish. twistMsg not valid.");
        }
    }

    void TeleoperationNode::keyCallback(const int key)
    {
        auto msg = input_mapper_.computeVelocity(key);
        if (msg) {
            pub_twist_.publish(*msg);
        }
        else {
            ROS_ERROR("TeleoperationNode::keyCallback(): Couldn t publish. twistMsg not valid.");
        }
    }

    TeleoperationNode::TeleoperationNode() {

        sub_joy_ = nh.subscribe("/joy", 1, &TeleoperationNode::joyCallback, this);
	    pub_twist_ = nh.advertise<geometry_msgs::Twist>("officerobot/cmd_vel", 1);

        if(!readParameters()) {

            ROS_ERROR("Node stopped because of missing parameters");
            return;
        }

        if(keyboard_enable_)
        {
            keyboard_input_.start();
        }
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "TeleoperationNode");

    officerobot::TeleoperationNode teleoperation_node;

    ros::spin();

    return 0;
}
