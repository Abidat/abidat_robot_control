# Abidat
## Office Robot

<br />

### OfficeRobot Tele Operation Node
***

Officerobot_tele_op node is used to remotely control the robot by using a regular keyboard or a controller (currently the node parameters - in param/tele_op.yaml file - are set for PS3 Controller. The node works also with different controller, but parameters need to be adjusted).

* Input of the node from controller -> joy message from the topic named */joy*
* Input of the node from keyboard -> remapped integer from getKey function
* Output of the node -> twist message to the topic named */officerobot/cmd_vel*

#### When using the controller input 

The node receives joy messages (through */joy* topic) from the controller and converts them into velocity (twist message to */officerobot/cmd_vel* topic). It reads the parameters from the parameter server, maps them in the input_mapper and, by using also the activation function, computes and publishes the final twist message.

* The parameters can be checked or modified in the *param/tele_op.yaml* parameter file.
* The activation function can be set in *param/tele_op.yaml*. It can be modified or a new one can be added  in the *src/library/activation_function.h* file.
* The device input file of the controller is by default set to /dev/input/js0. It can be checked in *launch/remote_control.launch*. If on your computer the device input file of the controller is different, it can be given as an in input when the node is launched:<br/>
`roslaunch officerobot remote_control.launch joy_device:=/dev/input/js*<number_of_your_controller_input_file>*`

#### When using the keyboard input

The node receives an integer value from the pressed arrow keys and converts them into velocity (twist message to */officerobot/cmd_vel* topic). It reads the parameters from the parameter server, maps them in the input_mapper, co,putes and publishes the final twist message.

* The parameters can be checked or modified in the *param/tele_op.yaml* parameter file.
* Keep in mind that *keyboard_enable* needs to be true in oder to use your keyboard.
* Right now you can only move into 4 directions and cant turn around.
* Use the arrow keys to control the robot.

#### How to start officerobot_tele_op node
In order to use the officerobot_tele_op node it must be started from a catkin workspace by using the corresponding launch file.
Steps:
1. Create a catkin workspace on your machine - let's name it *ros_ws*
2. Clone the officerobot repo inside the *ros_ws/src* 
3. Move to *ros_ws* folder and compile the project by using:<br/>
`cd ros_ws && catin_make`
4. Source the *devel/setup.bash* by using:<br/>
`source devel/setup.bash`
4. Actually start the node by executing: <br/>
`roslaunch officerobot remote_control.launch` (or the corresponding launch file name and using the needed parameters)