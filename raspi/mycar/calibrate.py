#!/usr/bin/env python3
"""
Scripts to drive a donkey 2 car

Usage:
    manage.py (drive)


Options:
    -h --help          Show this screen.
"""
import os
import time

from docopt import docopt

import donkeycar as dk

#import parts
from donkeycar.parts.controller import LocalWebController, \
    JoystickController, WebFpv
from donkeycar.parts.throttle_filter import ThrottleFilter
from donkeycar.parts import pins
from donkeycar.utils import *

from socket import gethostname



from docopt import docopt

import numpy as np
import pigpio
import time

#
# import cv2 early to avoid issue with importing after tensorflow
# see https://github.com/opencv/opencv/issues/14884#issuecomment-599852128
#
try:
    import cv2
except:
    pass

#Steering parts for TatamiRacer
class PWMSteering_TATAMI:
    def __init__(self,cfg):

        self.gpio_pin = 14 #Servo PWM pin
        self.pi = pigpio.pi()
        self.pigpio = pigpio
        
        #tatamiRacer Steering Control Tunable Parameter
        self.left_pulse = cfg.TATAMI_STEERING_LEFT_PWM #LEFT PWM
        self.right_pulse = cfg.TATAMI_STEERING_RIGHT_PWM #RIGHT PWM
        self.steering_feel = cfg.TATAMI_STEERING_FEEL #Steering Feeling Adjustment (Angle Level at Steering 50%)  
        self.steering_balance = cfg.TATAMI_STEERING_BALANCE #Steering L/R Balance -1.0(L)..+1.0(R)

        self.half_range = abs(self.right_pulse - self.left_pulse)/2
        self.center = min(self.left_pulse,self.right_pulse) + self.half_range

        self.servo_idle_time0 = time.time()
        self.servo_p0 = 0

        print('PWM Steering for TatamiRacer Created.')

    def update(self):
        pass
        
    def run_threaded(self, angle):
        
        #Steering Feeling Adjustment
        ang_abs=np.abs(angle)
        steering_half=0.5
        if ang_abs < steering_half:
            slope = self.steering_feel/steering_half
            angle = np.sign(angle)*ang_abs*slope
        else:
            slope = (1.0-self.steering_feel)/(1.0-steering_half)
            angle = np.sign(angle)* (self.steering_feel+(ang_abs-steering_half)*slope)
            
        #Steering Balance Adjustment
        if angle>0:
            angle =  angle * (1.0+self.steering_balance)
        else:
            angle =  angle * (1.0-self.steering_balance)
                
        #Steering PWM Calculation
        servo_p = int( self.center - self.half_range*angle  )
        if servo_p > self.left_pulse:
            servo_p = self.left_pulse
        elif servo_p < self.right_pulse:
            servo_p = self.right_pulse
        
        if self.servo_p0 != servo_p: #PWM not changed
            self.servo_idle_time0=time.time() #Count idle time
            servo_idle_time = 0
        else:
            servo_idle_time=time.time()-self.servo_idle_time0
        self.servo_p0 = servo_p
        
        if servo_idle_time <= 5.0:
            self.pi.set_servo_pulsewidth(self.gpio_pin, servo_p)
        else: #Servo power off
            self.pi.set_servo_pulsewidth(self.gpio_pin, 0)
            
    def run(self, angle):
        pass

    def shutdown(self):
        pi.set_mode(self.gpio_pin, self.pigpio.INPUT)
        self.pi.stop()      

#Throttle parts for TatamiRacer
class PWMThrottle_TATAMI:
    def __init__(self,cfg):
        self.gpio_pin0 = 13 #Motor PWM1 pin
        self.gpio_pin1 = 19 #Motor PWM2 pin
        self.pigpio = pigpio
        self.pi = pigpio.pi()

        #TatamiRacer Throttle Control Tunable Parameter
        self.throttle_start_boost_time = cfg.TATAMI_THROTTLE_START_BOOST_TIME
        self.throttle_start_boost = cfg.TATAMI_THROTTLE_START_BOOST
        self.throttle_upper_limit = cfg.TATAMI_THROTTLE_UPPER_LIMIT
        self.throttle_lower_limit = cfg.TATAMI_THROTTLE_LOWER_LIMIT
        self.throttle_steering_boost = cfg.TATAMI_THROTTLE_STEERING_BOOST
        
        self.throttle_deadzone = 0.01 #Throttle deadzone for detect zero (0..1)
        self.pwm_max = 100 #PWM Max
        self.pi.set_PWM_range(self.gpio_pin0,100)  # Set PWM range
        self.pi.set_PWM_frequency(self.gpio_pin0,490)
        self.pi.set_PWM_range(self.gpio_pin1,100)  # Set PWM range
        self.pi.set_PWM_frequency(self.gpio_pin1,490)
        
        self.throttle_start_boost_time0 = time.time()
        print('PWM Throttle for TatamiRacer created.')

    def update(self):
        pass

    def run_threaded(self, throttle,angle):
                    
        throttle_abs = np.abs(throttle)
        if throttle_abs<=self.throttle_deadzone:
            self.throttle_start_boost_time0 = time.time()
            throttle = 0.0

        #Throttle Boost
        t = time.time()-self.throttle_start_boost_time0
        if(t <= self.throttle_start_boost_time):
            boost = self.throttle_start_boost #Boost mode
        else:
            boost = 0.0

        #Steering Resistance Adjustment
        angle_adjust = self.throttle_lower_limit+np.abs(angle)*(self.throttle_steering_boost-self.throttle_lower_limit)

        #Throttle Feeling 
        if throttle_abs < self.throttle_lower_limit:
            throttle_feel = self.throttle_lower_limit
        elif throttle_abs > self.throttle_upper_limit:
            throttle_feel = self.throttle_upper_limit
        else:
            slope = self.throttle_upper_limit-self.throttle_lower_limit
            throttle_feel =self.throttle_lower_limit+ throttle_abs*slope
                                
        if throttle_abs > self.throttle_deadzone:
            throttle = np.sign(throttle)*max(boost,angle_adjust,throttle_feel)      
                
        #Set Motor PWM 
        motor_v = int(self.pwm_max*throttle)
        if np.abs(motor_v) > self.pwm_max:
            motor_v = int(np.sign(motor_v)*self.pwm_max)
        if throttle > 0:
            self.pi.set_PWM_dutycycle(self.gpio_pin0,motor_v) # Set PWM duty
            self.pi.set_PWM_dutycycle(self.gpio_pin1,0) # PWM off
        else:
            self.pi.set_PWM_dutycycle(self.gpio_pin1,-motor_v) # Set PWM duty
            self.pi.set_PWM_dutycycle(self.gpio_pin0,0) # PWM off
        
    def run(self, throttle):
        pass

    def shutdown(self):
        self.pi.set_PWM_dutycycle(self.gpio_pin0,0) # PWM off
        self.pi.set_PWM_dutycycle(self.gpio_pin1,0) # PWM off
        self.pi.stop()            




def drive(cfg ):
    '''
    Construct a working robotic vehicle from many parts.
    Each part runs as a job in the Vehicle loop, calling either
    it's run or run_threaded method depending on the constructor flag `threaded`.
    All parts are updated one after another at the framerate given in
    cfg.DRIVE_LOOP_HZ assuming each part finishes processing in a timely manner.
    Parts may have named outputs and inputs. The framework handles passing named outputs
    to parts requesting the same named input.
    '''

    #Initialize car
    V = dk.vehicle.Vehicle()

    ctr = LocalWebController(port=cfg.WEB_CONTROL_PORT)
    V.add(ctr,
          inputs=['cam/image_array', 'tub/num_records'],
          outputs=['angle', 'throttle', 'user/mode', 'recording'],
          threaded=True)

    #this throttle filter will allow one tap back for esc reverse
    th_filter = ThrottleFilter()
    V.add(th_filter, inputs=['throttle'], outputs=['throttle'])

    drive_train = None

    #Drive train setup
    if cfg.DONKEY_GYM or cfg.DRIVE_TRAIN_TYPE == "MOCK":
        pass

    elif cfg.DRIVE_TRAIN_TYPE == "PWM_STEERING_THROTTLE":
        #
        # drivetrain for RC car with servo and ESC.
        # using a PwmPin for steering (servo)
        # and as second PwmPin for throttle (ESC)
        #
        from donkeycar.parts.actuator import PWMSteering, PWMThrottle, PulseController
        dt = cfg.PWM_STEERING_THROTTLE
        steering_controller = PulseController(
            pwm_pin=pins.pwm_pin_by_id(dt["PWM_STEERING_PIN"]),
            pwm_scale=dt["PWM_STEERING_SCALE"],
            pwm_inverted=dt["PWM_STEERING_INVERTED"])
        steering = PWMSteering(controller=steering_controller,
                               left_pulse=dt["STEERING_LEFT_PWM"],
                               right_pulse=dt["STEERING_RIGHT_PWM"])

        throttle_controller = PulseController(
            pwm_pin=pins.pwm_pin_by_id(dt["PWM_THROTTLE_PIN"]),
            pwm_scale=dt["PWM_THROTTLE_SCALE"],
            pwm_inverted=dt['PWM_THROTTLE_INVERTED'])
        throttle = PWMThrottle(controller=throttle_controller,
                               max_pulse=dt['THROTTLE_FORWARD_PWM'],
                               zero_pulse=dt['THROTTLE_STOPPED_PWM'],
                               min_pulse=dt['THROTTLE_REVERSE_PWM'])

        drive_train = dict()
        drive_train['steering'] = steering
        drive_train['throttle'] = throttle

        V.add(steering, inputs=['angle'], threaded=True)
        V.add(throttle, inputs=['throttle'], threaded=True)

    elif cfg.DRIVE_TRAIN_TYPE == "I2C_SERVO":
        from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
        steering_controller = PCA9685(cfg.STEERING_CHANNEL,
                                      cfg.PCA9685_I2C_ADDR,
                                      busnum=cfg.PCA9685_I2C_BUSNUM)
        steering = PWMSteering(controller=steering_controller,
                               left_pulse=cfg.STEERING_LEFT_PWM,
                               right_pulse=cfg.STEERING_RIGHT_PWM)

        throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL,
                                      cfg.PCA9685_I2C_ADDR,
                                      busnum=cfg.PCA9685_I2C_BUSNUM)
        throttle = PWMThrottle(controller=throttle_controller,
                               max_pulse=cfg.THROTTLE_FORWARD_PWM,
                               zero_pulse=cfg.THROTTLE_STOPPED_PWM,
                               min_pulse=cfg.THROTTLE_REVERSE_PWM)

        drive_train = dict()
        drive_train['steering'] = steering
        drive_train['throttle'] = throttle
        V.add(steering, inputs=['angle'], threaded=True)
        V.add(throttle, inputs=['throttle'], threaded=True)

    elif cfg.DRIVE_TRAIN_TYPE == "PIGPIO_PWM":
        #
        # This driver is DEPRECATED in favor of 'DRIVE_TRAIN_TYPE == "PWM_STEERING_THROTTLE"'
        # This driver will be removed in a future release
        #
        from donkeycar.parts.actuator import PWMSteering, PWMThrottle, PiGPIO_PWM
        steering = PWMSteering_TATAMI(cfg)
        throttle = PWMThrottle_TATAMI(cfg)
        
        drive_train = dict()
        drive_train['steering'] = steering
        drive_train['throttle'] = throttle
        
        V.add(steering, inputs=['angle'], threaded=True)
        V.add(throttle, inputs=['throttle'], threaded=True)


    elif cfg.DRIVE_TRAIN_TYPE == "MM1":
        from donkeycar.parts.robohat import RoboHATDriver
        drive_train = RoboHATDriver(cfg)
        V.add(drive_train, inputs=['angle', 'throttle'])

    # TODO: monkeypatching is bad!!!
    ctr.drive_train = drive_train
    ctr.drive_train_type = cfg.DRIVE_TRAIN_TYPE

    print(f"Go to http://{gethostname()}.local:{ctr.port}/calibrate to "
          f"calibrate ")

    V.start(rate_hz=cfg.DRIVE_LOOP_HZ, max_loop_count=cfg.MAX_LOOPS)


if __name__ == '__main__':
    args = docopt(__doc__)
    cfg = dk.load_config()

    if args['drive']:
        drive(cfg)
