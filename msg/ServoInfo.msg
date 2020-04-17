duration since_start    # time passed since initialization
float32 maximum_voltage # highest legal voltage
float32 minimum_voltage # lowest legal voltage
int16 maximum_angle_deg # highest legal angle in degrees
int16 minimum_angle_deg # lowest legal angle in degrees
float32 maximum_temperature # highest legal temperature in Celsius
float32 input_voltage       # current input voltage
uint8 id                    # ID of the servo in use
float32 temperature         # current temperature in Celsius
int16 physical_position     # absolute physical position in degrees
int16 virtual_position      # absolute virtual position in degrees

uint8 mode
uint8 MODE_SERVO = 1    # servo mode: servomotor can be rotated within the givin range (minimum_angle_deg and maximum_angle_deg)
uint8 MODE_MOTOR = 2    # motor mode: servomotor will rotate constantly 360 degrees

uint8 LED
uint8 LED_ON=1          # LED on: LED on the servo is constantly active, which means that it can be used without problems
uint8 LED_BLINKING=2    # LED blinking: LED is blinking in order to alarm that there is something wrong, like maximum_temperature is exceeded or not enough voltage is given.