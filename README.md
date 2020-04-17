# Officerobot
This repository contains software to control our officerobots like [Marvin](https://github.com/Abidat/abidat_robot_construction). The application uses ROS framework and is separated into these three ROS nodes: 
* Forward Kinematics
* Teleoperation 
* Motor Handler

# Forward Kinematics Node

The forward kinematics node is used to publish and subscribe twist messages for the velocity of the omniwheel robot "marvin".

## Parameters

This are the parameters that are needed to calculate the forward kinematics for the omniwheel robot.

    distanceWheels: <double>
The distance between the single wheels in meter. The diagonal between two wheels is not included here.

    wheelDiameter: <double>
The diameter of each single wheel in meter.

## Topics

This is the ros topic which the node should publish the motor control for each motor.

    publisher_topic: "<topic>"

This is the ros topic which the node should subscribe to the velocity.

    subscriber_topic: "<topic>"


## Testing

To test if the node is publishing and subscribing correctly you can run this command:

    rostest officerobot basic_function_test.launch

===========================

#  Teleoperation Node

Teleoperation node is used to remotely control the robot by using a regular keyboard or a controller (currently the node parameters - in param/tele_op.yaml file - are set for PS3 Controller. The node works also with different controllers, but parameters need to be adjusted).

## Parameters

    keyboard_enable: <bool>
enables or disables the ability to use a keyboard to control the robot.

    linear_keyboard_speed: <velocity>
sets a fix velocity for linear movement for keyboard input. Use floats between 0 and 1

    index_linear_speed_x: 1
Position in the `int32[] buttons` vector of `Joy message`. At this position is received and read the linear speed of the robot for x axis.

    index_linear_speed_y: 0
Position in the `int32[] buttons` vector of `Joy message`. At this position is received and read the linear speed of the robot for x axis.

    index_angular_speed: 3 
Position in the `int32[] buttons` vector of `Joy message`. At this position is received and read the angular speed of the robot.

    max_linear_speed: 10
The maximum linear speed the robot can reach in meters per second.

    max_angular_speed: 10
The maximum angular speed the robot can reach in radian per second.

    activation_function: "linear"
The type of activation function that is used. Possible values: linear, exponential

## start teleop node

To use the control the robot remotely we use the teleoperation node. Start it with:

    roslaunch officerobot remote_control.launch

## used topics

To communicate with the teleoperation node, there are 2 given topics to speak and listen to.

When used, the teleoperation node subscribes to this topic where, controllers like PS3 controllers publish to. It receives a joy message there.

    /joy

After getting a joy message or a keyboard input the teleoperation node remaps the massage and publishes a twist message to this topic:

    /officerobot/cmd_vel

## debugging

To catch and see the published twist messages use the following command:

    rostopic echo /officerobot/cmd_vel

# Servo Control Node

The servo control node is used to communicate and control the servomotor Lewan Soul LX-16A. It is able to change its angle and give information about his current status. Please follow these instructions to use it properly!

## Hardware setup

First you need to setup the LX-16A servo by connecting it to the circuit board. It is possible to connect more servos but till now this program only works with one.
After connecting the servo to it, it is needed to connect a power supply with at least <volt> and at most <volt> voltage and to the device wanted to control it with, via usb-c connection.

## Software preperation

In order to prepare the Linux OS for this program please execute [udev.sh](./udev/udev.sh).<br/>
Now the OS is ready and no further dependencies are required.

## Parameters

    servo_id: <ID>
ID of the servomotor in use

    servo_port: "<device>"
used file descriptor for usb servo control device

    publisher_topic: "<topic>"
ros topic which the node should publish the servos status

    subscriber_topic: "<topic>"
ros topic which the node should recieve the new angles

## start servo control node

To use the servos on the robot we use the servocontrol node. Start it with:

    roslaunch officerobot servo.launch

## used topics

To communitcate with the servomotor, there are 2 given topics to speak and listen to.

The servo subscribes to the following topic and needs an integer value between 0 and 240 to move to the ordered angle

    /servo/degree

The servo is constantly publishing its own status with static and dynamic values to this topic:

    /servo/info

## debugging

In order to get move the servomotor into your wished angle, you should use the following command:
    
    rostopic pub /servo/degree std_msgs/Int64 "data: <angle>"
please make sure to replace "angle" with a interger value between 0 and 240

To get the current status of the servomotor use:

    rostopic echo /servo/info

