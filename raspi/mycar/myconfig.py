MM1_STEERING_MID = 1550
MM1_MAX_FORWARD = 1620  # Max is 2000
MM1_MAX_REVERSE = 1350
MM1_STOPPED_PWM = 1500
MM1_SHOW_STEERING_VALUE = False
MM1_SERIAL_PORT = '/dev/ttyS0'

THROTTLE_FORWARD_PWM = 410  # pwm value for max forward throttle
THROTTLE_STOPPED_PWM = 370  # pwm value for no movement
THROTTLE_REVERSE_PWM = 320  # pwm value for max reverse throttle

# SERVO_ESC|DC_STEER_THROTTLE|DC_TWO_WHEEL|DC_TWO_WHEEL_L298N|SERVO_HBRIDGE_PWM|PIGPIO_PWM|MM1|MOCK
DRIVE_TRAIN_TYPE = "PIGPIO_PWM"

JOYSTICK_MAX_THROTTLE = 1.0
JOYSTICK_THROTTLE_DIR = -1.0

CONTROLLER_TYPE = 'F710'  # (ps3|ps4)
DRIVE_LOOP_HZ = 20

if (CONTROLLER_TYPE == 'F710'):
    JOYSTICK_DEADZONE = 0.1

AUTO_CREATE_NEW_TUB = True
MAX_EPOCHS = 30

USE_SSD1306_128_32 = False  # Enable the SSD_1306 OLED Display

# DONKEY_GYM = True
# DONKEY_SIM_PATH = "remote"
# SIM_HOST="192.168.1.238"

CAMERA_VFLIP = True
CAMERA_HFLIP = True
STEERING_LEFT_PWM = 460.0
STEERING_RIGHT_PWM = 290.0

#TatamiRacer Tunable Parameter (TATAMI_*)
TATAMI_STEERING_LEFT_PWM = 2172 #PWM value for full left steering (Center PWM + steering level)
TATAMI_STEERING_RIGHT_PWM = 916 #PWM value for full right steering
TATAMI_STEERING_FEEL = 0.3 #Steering Feeling Adjustment (Angle Level at Steering 50%)  
TATAMI_STEERING_BALANCE = -0.1 #Steering L/R Balance -1.0(L)..+1.0(R)
TATAMI_THROTTLE_START_BOOST_TIME = 0.9 #Throttle boost time[sec]
TATAMI_THROTTLE_START_BOOST = 1.0 #Throttle boost level for start torque up(0..1)
TATAMI_THROTTLE_UPPER_LIMIT = 1.0 #Throttle upper limit (0..1)
TATAMI_THROTTLE_LOWER_LIMIT = 0.8 #Throttle lower limit (0..1)
TATAMI_THROTTLE_STEERING_BOOST = 1.0 #Throttle boost adjustment by steering angle (0..1)