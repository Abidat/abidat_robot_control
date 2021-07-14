#!/usr/bin/env python3
# ROS dependencies
import rospy
from std_msgs.msg import Int64
from abidat_robot_control.msg import ServoInfo
import time

# servo lib dependency
from lx16a import LX16A

class ServoStaticStatus:
    """Saves the constant status values of the servomotor"""
    def __init__(self):
        self.max_voltage = 0.0
        self.min_voltage = 0.0
        self.min_angle = 0
        self.max_angle = 0
        self.max_temperature = 0.0

class ServoDynamicStatus:
    """Saves the dynamic status values of the servomotor"""
    def __init__(self):
        self.input_voltage = 0.0
        self.offset_angle = 0.0
        self.id = 0
        self.temperature = 0.0
        self.physical_pos = 0
        self.virtual_pos = 0
        self.mode = 0
        self.loaded = 0
        self.LED = 0
        self.LED_error = 0, 0, 0

class Parameters:
    """Read parameters from the parameter server"""
    def __init__(self):
        self.servo_id = None
        self.servo_port = None
        self.pub_topic = None
        self.sub_topic = None
    
def readParam() -> Parameters:
    """Read parameters from the parameter server
    
    Returns:
    param - read parameters from the parameterserver and save them in Parameters class"""
    param = Parameters()
    param.servo_id = rospy.get_param('~servo_id')
    param.servo_port = rospy.get_param('~servo_port')
    param.pub_topic = rospy.get_param('~publisher_topic')
    param.sub_topic = rospy.get_param('~subscriber_topic')

    return param

def moveServo(data: int):
    """Rotates the servomotor to an absolute degree
    
    Keyword arguments:
    data -- integer degree given by topic"""
    servomotor.moveTimeWrite(data.data)


def getAndPublishServoStatus(my_time: time):
    """Requests static and dynamic Statusinformation about the servomotor before returning it
    
    Keyword arguments:
    my_time -- time stamp from first call of this function
    Returns:
    Stored status from staticStatusRequest() and dynamicStatusRequest()"""
    staticStatus = staticStatusRequest()
    dynamicStatus = dynamicStatusRequest()

    message = ServoInfo()
    message.since_start = rospy.get_rostime() - servo_time

    # publishing static information
    message.maximum_voltage     = staticStatus.max_voltage
    message.minimum_voltage     = staticStatus.min_voltage
    message.minimum_angle_deg   = staticStatus.min_angle
    message.maximum_angle_deg   = staticStatus.max_angle
    message.maximum_temperature = staticStatus.max_temperature

    # publishing dynamic information
    message.input_voltage     = dynamicStatus.input_voltage
    message.id                = dynamicStatus.id
    message.temperature       = dynamicStatus.temperature
    message.physical_position = dynamicStatus.physical_pos
    message.virtual_position  = dynamicStatus.virtual_pos

    if dynamicStatus.mode == 0:
        message.mode = message.MODE_SERVO
    else:
        message.mode = message.MODE_MOTOR
    if dynamicStatus.LED == 0:
        message.LED = message.LED_ON
    else:
        message.LED = message.LED_BLINKING
        servo_pub.publish(message)

def initPublisher(topic: str):
    """initialize publisher
    
    Keyword arguments:
    topic -- ROS topic information should be published to"""
    rospy.loginfo("initializing status publisher")
    global servo_pub
    servo_pub = rospy.Publisher(topic, ServoInfo, queue_size = 1)
    rospy.loginfo("initialization successful")

def initNodeAndServo():
    """Mainfunction 
    
    Initializing ROS node and servomotors"""
    rospy.init_node('servocontrol', anonymous=True)
    param = readParam()
    initServoBus(param.servo_port)
    initServo(param.servo_id)

    initPublisher(param.pub_topic)
    initSubscriber(param.sub_topic)

    global servo_time
    servo_time = rospy.get_rostime()
    rospy.Timer(rospy.Duration(1), getAndPublishServoStatus)

    rospy.spin()

def initServoBus(device: str):
    """Initialize servobus
    
    Keyword arguments:
    device -- connected USB port"""
    rospy.loginfo("initializing servo bus on " + str(device))
    LX16A.initialize(device)
    rospy.loginfo("initialization successful")

def initServo(sid: int):
    """Initialize servomotor

    Keyword arguments:
    sid -- id of chosen servomotor"""
    rospy.loginfo("initializing servo" + str(sid))
    global servomotor
    servomotor = LX16A(sid)
    rospy.loginfo("initialization successful")

    

def initSubscriber(topic: str):
    """Initialize subscriber to given topic
    
    Keyword arguments:
    topic -- ROS topic to subscribe to"""
    rospy.loginfo("initializing ROS subscriber to " + str(topic))
    rospy.Subscriber(topic, Int64, moveServo)
    rospy.loginfo("initialization successful")

def staticStatusRequest() -> ServoStaticStatus:
    """Gets the static status of the servomotor.

    Returns:
    status of given servo stored (ServoStaticStatus)
    """
    status = ServoStaticStatus()
           
    status.min_angle, status.max_angle = servomotor.angleLimitRead()    
    VLimitMin, VLimitMax = servomotor.vInLimitRead()
    status.max_voltage = VLimitMax / 1000
    status.min_voltage = VLimitMin /1000
    status.max_temperature = servomotor.tempMaxLimitRead()

    return status

def dynamicStatusRequest() -> ServoDynamicStatus:
    """Gets the dynamic status of the servomotor.
    
    Returns:
    status of servo stored (ServoDynamicStatus)
    """
    status = ServoDynamicStatus()
    
    status.input_voltage = servomotor.vInRead() / 1000
    status.offset_angle = servomotor.angleOffsetRead()
    status.id = servomotor.IDRead()
    status.temperature = servomotor.tempRead()
    status.physical_pos = servomotor.getPhysicalPos()
    status.virtual_pos = servomotor.getVirtualPos()
    status.mode = servomotor.servoMotorModeRead()
    status.loaded = servomotor.loadOrUnloadRead()
    status.LED = servomotor.LEDCtrlRead()
    status.LED = servomotor.LEDErrorRead()

    return status

if __name__ == '__main__':
    """Defines the main function"""
    initNodeAndServo()
