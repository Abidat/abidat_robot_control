#!/usr/bin/env python3
# ROS dependencies
import rospy
from std_msgs.msg import Int64
from officerobot.msg import ServoInfo
from functools import partial
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

class ServoParameters:
    """Read parameters from the parameter server"""
    def __init__(self):
        self.ID = None
        self.servo_port = None
        self.pub_topic = None
        self.sub_topic = None
        self.min_angle = None
        self.max_angle = None
    
def readParam(servo: str): #-> params:list, servo_port:str
    """Read parameters from the parameter server
    
    Returns:
    param - read parameters from the parameterserver and save them in Parameters class"""
    # servos = rospy.get_param('~servos')
    # servo_port = rospy.get_param('~servo_port')

    #params = []
    param = ServoParameters()
    param.ID = rospy.get_param('~' + str(servo) + '/ID')
    param.pub_topic = rospy.get_param('~' + str(servo) + '/publisher_topic')
    param.sub_topic = rospy.get_param('~' + str(servo) + '/subscriber_topic')
    param.min_angle = rospy.get_param('~' + str(servo) + '/min_angle')
    param.max_angle = rospy.get_param('~' + str(servo) + '/max_angle')
    #params.append(param)

    return param#, servo_port

def moveServo(servo:int, data: int):
    """Rotates the servomotor to an absolute degree
    
    Keyword arguments:
    data -- integer degree given by topic"""
    servomotor[servo].moveTimeWrite(data.data)


def getAndPublishServoStatus(servo:int, my_time: time):
    """Requests static and dynamic Statusinformation about the servomotor before returning it
    
    Keyword arguments:
    my_time -- time stamp from first call of this function
    Returns:
    Stored status from staticStatusRequest() and dynamicStatusRequest()"""
    staticStatus = staticStatusRequest(servo)
    dynamicStatus = dynamicStatusRequest(servo)

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
    servos = rospy.get_param('~servos')
    servo_port = rospy.get_param('~servo_port')
    initServoBus(servo_port)
    global servomotor
    servomotor = {}
    for servo in servos:
        param = readParam(servo)
        servomotor[servo] = initServo(param.ID, servo)
        configAngle(param.min_angle, param.max_angle, servo)
        initPublisher(param.pub_topic)
        initSubscriber(param.sub_topic, servo)
        global servo_time
        servo_time = rospy.get_rostime()
        rospy.Timer(rospy.Duration(1), partial(getAndPublishServoStatus, servo))
    # TODO Callbacks (getAndPublishServoStatus and moveServo) needs to get the directory key!!!
    # ERROR doesn't save the first motor (servo_a)
    rospy.spin()

def initServoBus(device: str):
    """Initialize servobus
    
    Keyword arguments:
    device -- connected USB port"""
    rospy.loginfo("initializing servo bus on " + str(device))
    LX16A.initialize(device)
    rospy.loginfo("initialization successful")

def initServo(sid: int, servo:str):
    """Initialize servomotor

    Keyword arguments:
    sid -- id of chosen servomotor"""
    rospy.loginfo("initializing servo" + str(sid))
    motor = LX16A(sid)
    rospy.loginfo("initialization successful")

    return motor

def configAngle(min_angle: int, max_angle: int, servo: str):
    """Sets minimum and maximum angle for the servomotor
    
    Keyword arguments:
    min_angle -- minimum angle received as a parameter
    max_angle -- maximum angle received as a parameter"""
    servomotor[servo].angleLimitWrite(min_angle, max_angle)

def initSubscriber(topic: str, servo: str):
    """Initialize subscriber to given topic and calls moveServo when input is given
    
    Keyword arguments:
    topic -- ROS topic to subscribe to"""
    rospy.loginfo("initializing ROS subscriber to " + str(topic))
    rospy.Subscriber(topic, Int64, partial(moveServo, servo))
    rospy.loginfo("initialization successful")

def staticStatusRequest(servo: str) -> ServoStaticStatus:
    """Gets the static status of the servomotor.

    Returns:
    status of given servo stored (ServoStaticStatus)
    """
    status = ServoStaticStatus()
           
    status.min_angle, status.max_angle = servomotor[servo].angleLimitRead()    
    VLimitMin, VLimitMax = servomotor[servo].vInLimitRead()
    status.max_voltage = VLimitMax / 1000
    status.min_voltage = VLimitMin /1000
    status.max_temperature = servomotor[servo].tempMaxLimitRead()

    return status

def dynamicStatusRequest(servo: str) -> ServoDynamicStatus:
    """Gets the dynamic status of the servomotor.
    
    Returns:
    status of servo stored (ServoDynamicStatus)
    """
    status = ServoDynamicStatus()
    
    status.input_voltage = servomotor[servo].vInRead() / 1000
    status.offset_angle = servomotor[servo].angleOffsetRead()
    status.id = servomotor[servo].IDRead()
    status.temperature = servomotor[servo].tempRead()
    status.physical_pos = servomotor[servo].getPhysicalPos()
    status.virtual_pos = servomotor[servo].getVirtualPos()
    status.mode = servomotor[servo].servoMotorModeRead()
    status.loaded = servomotor[servo].loadOrUnloadRead()
    status.LED = servomotor[servo].LEDCtrlRead()
    status.LED = servomotor[servo].LEDErrorRead()

    return status

if __name__ == '__main__':
    """Defines the main function"""
    initNodeAndServo()
