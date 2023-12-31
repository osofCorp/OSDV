"""
actuators.py
Classes to control the motors and servos. These classes 
are wrapped in a mixer class before being used in the drive loop.
"""

from abc import ABC, abstractmethod
import time
import logging
from typing import Tuple

import donkeycar as dk
from donkeycar import utils
from donkeycar.utils import clamp

logger = logging.getLogger(__name__)

try:
    import RPi.GPIO as GPIO
except ImportError as e:
    logger.warn(f"RPi.GPIO was not imported. {e}")
    globals()["GPIO"] = None

from donkeycar.parts.pins import OutputPin, PwmPin, PinState
from donkeycar.utilities.deprecated import deprecated

logger = logging.getLogger(__name__)


#
# pwm/duty-cycle/pulse
# - Standard RC servo pulses range from 1 millisecond (full reverse)
#   to 2 milliseconds (full forward) with 1.5 milliseconds being neutral (stopped).
# - These pulses are typically send at 50 hertz (every 20 milliseconds).
# - This means that, using the standard 50hz frequency, a 1 ms pulse
#   represents a 5% duty cycle and a 2 ms pulse represents a 10% duty cycle.
# - The important part is the length of the pulse;
#   it must be in the range of 1 ms to 2ms.
# - So this means that if a different frequency is used, then the duty cycle
#   must be adjusted in order to get the 1ms to 2ms pulse.
# - For instance, if a 60hz frequency is used, then a 1 ms pulse requires
#   a duty cycle of 0.05 * 60 / 50 = 0.06 (6%) duty cycle
# - We default the frequency of our PCA9685 to 60 hz, so pulses in
#   config are generally based on 60hz frequency and 12 bit values.
#   We use 12 bit values because the PCA9685 has 12 bit resolution.
#   So a 1 ms pulse is 0.06 * 4096 ~= 246, a neutral pulse of 0.09 duty cycle
#   is 0.09 * 4096 ~= 367 and full forward pulse of 0.12 duty cycles
#   is 0.12 * 4096 ~= 492
# - These are generalizations that are useful for understanding the underlying
#   api call arguments.  The final choice of duty-cycle/pulse length depends
#   on your hardware and perhaps your strategy (you may not want to go too fast,
#   and so you may choose is low max throttle pwm)
#

def duty_cycle(pulse_ms:float, frequency_hz:float) -> float:
    """
    Calculate the duty cycle, 0 to 1, of a pulse given
    the frequency and the pulse length

    :param pulse_ms:float the desired pulse length in milliseconds
    :param frequency_hz:float the pwm frequency in hertz
    :return:float duty cycle in range 0 to 1
    """
    ms_per_cycle = 1000 / frequency_hz
    duty = pulse_ms / ms_per_cycle
    return duty


def pulse_ms(pulse_bits:int) -> float:
    """
    Calculate pulse width in milliseconds given a
    12bit pulse (as a PCA9685 would use).
    Donkeycar throttle and steering PWM values are
    based on PCA9685 12bit pulse values, where
    0 is zero duty cycle and 4095 is 100% duty cycle.

    :param pulse_bits:int 12bit integer in range 0 to 4095
    :return:float pulse length in milliseconds
    """
    if pulse_bits < 0 or pulse_bits > 4095:
        raise ValueError("pulse_bits must be in range 0 to 4095 (12bit integer)")
    return pulse_bits / 4095


class PulseController:
    """
    Controller that provides a servo PWM pulse using the given PwmPin
    See pins.py for pin provider implementations.
    """

    def __init__(self, pwm_pin:PwmPin, pwm_scale:float = 1.0, pwm_inverted:bool = False) -> None:
        """
        :param pwm_pin:PwnPin pin that will emit the pulse.
        :param pwm_scale:float scaling the 12 bit pulse value to compensate
                        for non-standard pwm frequencies.
        :param pwm_inverted:bool True to invert the duty cycle
        """
        self.pwm_pin = pwm_pin
        self.scale = pwm_scale
        self.inverted = pwm_inverted
        self.started = pwm_pin.state() != PinState.NOT_STARTED

    def set_pulse(self, pulse:int) -> None:
        """
        Set the length of the pulse using a 12 bit integer (0..4095)
        :param pulse:int 12bit integer (0..4095)
        """
        if pulse < 0 or pulse > 4095:
            logging.error("pulse must be in range 0 to 4095")
            pulse = clamp(pulse, 0, 4095)

        if not self.started:
            self.pwm_pin.start()
            self.started = True
        if self.inverted:
            pulse = 4095 - pulse
        self.pwm_pin.duty_cycle(int(pulse * self.scale) / 4095)

    def run(self, pulse:int) -> None:
        """
        Set the length of the pulse using a 12 bit integer (0..4095)
        :param pulse:int 12bit integer (0..4095)
        """
        self.set_pulse(pulse)


        
class PCA9685:
    ''' 
    PWM motor controler using PCA9685 boards. 
    This is used for most RC Cars
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None, init_delay=0.1):

        self.default_freq = 60
        self.pwm_scale = frequency / self.default_freq

        import Adafruit_PCA9685
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            # replace the get_bus function with our own
            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        time.sleep(init_delay) # "Tamiya TBLE-02" makes a little leap otherwise

    def set_pulse(self, pulse):
        try:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))
        except:
            self.pwm.set_pwm(self.channel, 0, int(pulse * self.pwm_scale))

    def run(self, pulse):
        self.set_pulse(pulse)


class PiGPIO_PWM():
    '''
    # Use the pigpio python module and daemon to get hardware pwm controls from
    # a raspberrypi gpio pins and no additional hardware. Can serve as a replacement
    # for PCA9685.
    #
    # Install and setup:
    # sudo apt update && sudo apt install pigpio python3-pigpio
    # sudo systemctl start pigpiod
    #
    # Note: the range of pulses will differ from those required for PCA9685
    # and can range from 12K to 170K
    #
    # If you use a control circuit that inverts the steering signal, set inverted to True
    # Default multipler for pulses from config etc is 100
    '''

    def __init__(self, pin, pgio=None, freq=75, inverted=False):
        import pigpio
        self.pin = pin
        self.pgio = pgio or pigpio.pi()
        self.freq = freq
        self.inverted = inverted
        self.pgio.set_mode(self.pin, pigpio.OUTPUT)
        self.dead_zone = 37000

    def __del__(self):
        self.pgio.stop()

    def set_pulse(self, pulse):
#
        self.output = pulse * 200
        if self.output > 0:
            self.pgio.hardware_PWM(self.pin, self.freq, int(self.output if self.inverted == False else 1e6 - self.output))


    def run(self, pulse):
        self.set_pulse(pulse)

class PWMSteering:
    """
    Wrapper over a PWM motor controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self,
                 controller=None,
                 left_pulse=290,
                 right_pulse=490):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.pulse = dk.utils.map_range(0, self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)
        self.running = True
        print('PWM Steering created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, angle):
        # map absolute angle to angle that vehicle can implement.
        self.pulse = dk.utils.map_range(angle,
                                        self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)

    def run(self, angle):
        self.run_threaded(angle)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # set steering straight
        self.pulse = 0
        time.sleep(0.3)
        self.running = False


class PWMThrottle:
    """
    Wrapper over a PWM motor controller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self,
                 controller=None,
                 max_pulse=300,
                 min_pulse=490,
                 zero_pulse=350):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.pulse = zero_pulse

        # send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_pulse(self.max_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.min_pulse)
        time.sleep(0.01)
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)
        self.running = True
        print('PWM Throttle created')

    def update(self):
        while self.running:
            self.controller.set_pulse(self.pulse)

    def run_threaded(self, throttle):
        if throttle > 0:
            self.pulse = dk.utils.map_range(throttle, 0, self.MAX_THROTTLE,
                                            self.zero_pulse, self.max_pulse)
        else:
            self.pulse = dk.utils.map_range(throttle, self.MIN_THROTTLE, 0,
                                            self.min_pulse, self.zero_pulse)

    def run(self, throttle):
        self.run_threaded(throttle)
        self.controller.set_pulse(self.pulse)

    def shutdown(self):
        # stop vehicle
        self.run(0)
        self.running = False

class JHat:
    ''' 
    PWM motor controler using Teensy emulating PCA9685. 
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        print("Firing up the Hat")
        import Adafruit_PCA9685
        LED0_OFF_L = 0x08
        # Initialise the PCA9685 using the default address (0x40).
        if busnum is not None:
            from Adafruit_GPIO import I2C
            # replace the get_bus function with our own
            def get_bus():
                return busnum
            I2C.get_default_bus = get_bus
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.channel = channel
        self.register = LED0_OFF_L+4*channel

        # we install our own write that is more efficient use of interrupts
        self.pwm.set_pwm = self.set_pwm
        
    def set_pulse(self, pulse):
        self.set_pwm(self.channel, 0, pulse) 

    def set_pwm(self, channel, on, off):
        # sets a single PWM channel
        self.pwm._device.writeList(self.register, [off & 0xFF, off >> 8])
        
    def run(self, pulse):
        self.set_pulse(pulse)

class JHatReader:
    ''' 
    Read RC controls from teensy 
    '''
    def __init__(self, channel, address=0x40, frequency=60, busnum=None):
        import Adafruit_PCA9685
        self.pwm = Adafruit_PCA9685.PCA9685(address=address)
        self.pwm.set_pwm_freq(frequency)
        self.register = 0 #i2c read doesn't take an address
        self.steering = 0
        self.throttle = 0
        self.running = True
        #send a reset
        self.pwm._device.writeRaw8(0x06)

    def read_pwm(self):
        '''
        send read requests via i2c bus to Teensy to get
        pwm control values from last RC input  
        '''
        h1 = self.pwm._device.readU8(self.register)
        # first byte of header must be 100, otherwize we might be reading
        # in the wrong byte offset
        while h1 != 100:
            print("skipping to start of header")
            h1 = self.pwm._device.readU8(self.register)
        
        h2 = self.pwm._device.readU8(self.register)
        # h2 ignored now

        val_a = self.pwm._device.readU8(self.register)
        val_b = self.pwm._device.readU8(self.register)
        self.steering = (val_b << 8) + val_a
        
        val_c = self.pwm._device.readU8(self.register)
        val_d = self.pwm._device.readU8(self.register)
        self.throttle = (val_d << 8) + val_c

        # scale the values from -1 to 1
        self.steering = (self.steering - 1500.0) / 500.0  + 0.158
        self.throttle = (self.throttle - 1500.0) / 500.0  + 0.136

    def update(self):
        while(self.running):
            self.read_pwm()
        
    def run_threaded(self):
        return self.steering, self.throttle

    def shutdown(self):
        self.running = False
        time.sleep(0.1)





class Adafruit_DCMotor_Hat:
    ''' 
    Adafruit DC Motor Controller 
    Used for each motor on a differential drive car.
    '''
    def __init__(self, motor_num):
        from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
        import atexit
        
        self.FORWARD = Adafruit_MotorHAT.FORWARD
        self.BACKWARD = Adafruit_MotorHAT.BACKWARD
        self.mh = Adafruit_MotorHAT(addr=0x60) 
        
        self.motor = self.mh.getMotor(motor_num)
        self.motor_num = motor_num
        
        atexit.register(self.turn_off_motors)
        self.speed = 0
        self.throttle = 0
    
        
    def run(self, speed):
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        self.throttle = int(dk.utils.map_range(abs(speed), -1, 1, -255, 255))
        
        if speed > 0:            
            self.motor.run(self.FORWARD)
        else:
            self.motor.run(self.BACKWARD)
            
        self.motor.setSpeed(self.throttle)
        

    def shutdown(self):
        self.mh.getMotor(self.motor_num).run(Adafruit_MotorHAT.RELEASE)


class Maestro:
    '''
    Pololu Maestro Servo controller
    Use the MaestroControlCenter to set the speed & acceleration values to 0!
    '''
    import threading

    maestro_device = None
    astar_device = None
    maestro_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Maestro.maestro_device == None:
            Maestro.maestro_device = serial.Serial('/dev/ttyACM0', 115200)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Maestro.astar_device == None:
            Maestro.astar_device = serial.Serial('/dev/ttyACM2', 115200, timeout= 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds
        w *= 4  # in quarter microsenconds the maestro wants
        w = int(w)

        with Maestro.maestro_lock:
            Maestro.maestro_device.write(bytearray([ 0x84,
                                                     self.channel,
                                                     (w & 0x7F),
                                                     ((w >> 7) & 0x7F)]))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Maestro.astar_lock:
                Maestro.astar_device.write(b)

    def readline(self):
        ret = None
        with Maestro.astar_lock:
            # expecting lines like
            # E n nnn n
            if Maestro.astar_device.inWaiting() > 8:
                ret = Maestro.astar_device.readline()

        if ret is not None:
            ret = ret.rstrip()

        return ret


class Teensy:
    '''
    Teensy Servo controller
    '''
    import threading

    teensy_device = None
    astar_device = None
    teensy_lock = threading.Lock()
    astar_lock = threading.Lock()

    def __init__(self, channel, frequency = 60):
        import serial

        if Teensy.teensy_device == None:
            Teensy.teensy_device = serial.Serial('/dev/teensy', 115200, timeout = 0.01)

        self.channel = channel
        self.frequency = frequency
        self.lturn = False
        self.rturn = False
        self.headlights = False
        self.brakelights = False

        if Teensy.astar_device == None:
            Teensy.astar_device = serial.Serial('/dev/astar', 115200, timeout = 0.01)

    def set_pulse(self, pulse):
        # Recalculate pulse width from the Adafruit values
        w = pulse * (1 / (self.frequency * 4096)) # in seconds
        w *= 1000 * 1000  # in microseconds

        with Teensy.teensy_lock:
            Teensy.teensy_device.write(("%c %.1f\n" % (self.channel, w)).encode('ascii'))

    def set_turn_left(self, v):
        if self.lturn != v:
            self.lturn = v
            b = bytearray('L' if v else 'l', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_turn_right(self, v):
        if self.rturn != v:
            self.rturn = v
            b = bytearray('R' if v else 'r', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_headlight(self, v):
        if self.headlights != v:
            self.headlights = v
            b = bytearray('H' if v else 'h', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def set_brake(self, v):
        if self.brakelights != v:
            self.brakelights = v
            b = bytearray('B' if v else 'b', 'ascii')
            with Teensy.astar_lock:
                Teensy.astar_device.write(b)

    def teensy_readline(self):
        ret = None
        with Teensy.teensy_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.teensy_device.inWaiting() > 8:
                ret = Teensy.teensy_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

    def astar_readline(self):
        ret = None
        with Teensy.astar_lock:
            # expecting lines like
            # E n nnn n
            if Teensy.astar_device.inWaiting() > 8:
                ret = Teensy.astar_device.readline()

        if ret != None:
            ret = ret.rstrip()

        return ret

class MockController(object):
    def __init__(self):
        pass

    def run(self, pulse):
        pass

    def shutdown(self):
        pass


class L298N_HBridge_2pin(object):


    def __init__(self, pin_forward:PwmPin, pin_backward:PwmPin, zero_throttle:float=0, max_duty = 0.9):
        """
        pin_forward:PwmPin Takes a duty cycle in the range of 0 to 1,
                        where 0 is fully off and 1 is fully on.
                        When the duty_cycle > 0 the motor will turn clockwise
                        proportial to the duty_cycle
        pin_backward:PwmPin Takes a duty cycle in the range of 0 to 1,
                            where 0 is fully off and 1 is fully on.
                            When the duty_cycle > 0 the motor will turn counter-clockwise
                            proportial to the duty_cycle
        zero_throttle: values at or below zero_throttle are treated as zero.
        max_duty: the maximum duty cycle that will be send to the motors

        NOTE: if pin_forward and pin_backward are both at duty_cycle == 0,
            then the motor is 'detached' and will glide to a stop.
            if pin_forward and pin_backward are both at duty_cycle == 1,
            then the motor will be forcibly stopped (can be used for braking)
        max_duty is from 0 to 1 (fully off to fully on). I've read 0.9 is a good max.
        """
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.zero_throttle = zero_throttle
        self.max_duty = max_duty

        self.throttle=0
        self.speed=0
        
        self.pin_forward.start(0)
        self.pin_backward.start(0)

    def run(self, throttle:float) -> None:
        """
        Update the speed of the motor
        :param throttle:float throttle value in range -1 to 1,
                        where 1 is full forward and -1 is full backwards.
        """
        if throttle is None:
            logger.warn("TwoWheelSteeringThrottle throttle is None")
            return
        if throttle > 1 or throttle < -1:
            logger.warn( f"TwoWheelSteeringThrottle throttle is {throttle}, but it must be between 1(forward) and -1(reverse)")
            throttle = clamp(throttle, -1, 1)

        self.speed = throttle
        self.throttle = dk.utils.map_range_float(throttle, -1, 1, -self.max_duty, self.max_duty)
        
        if self.throttle > self.zero_throttle:
            self.pin_backward.duty_cycle(0)
            self.pin_forward.duty_cycle(self.throttle)
        elif self.throttle < -self.zero_throttle:
            self.pin_forward.duty_cycle(0)
            self.pin_backward.duty_cycle(-self.throttle)
        else:
            self.pin_forward.duty_cycle(0)
            self.pin_backward.duty_cycle(0)

    def shutdown(self):
        self.pin_forward.stop()
        self.pin_backward.stop()


class TwoWheelSteeringThrottle(object):

    def run(self, throttle, steering):
        if throttle > 1 or throttle < -1:
            raise ValueError( "throttle must be between 1(forward) and -1(reverse)")
 
        if steering > 1 or steering < -1:
            raise ValueError( "steering must be between 1(right) and -1(left)")

        left_motor_speed = throttle
        right_motor_speed = throttle
 
        if steering < 0:
            left_motor_speed *= (1.0 - (-steering))
        elif steering > 0:
            right_motor_speed *= (1.0 - steering)

        return left_motor_speed, right_motor_speed

    def shutdown(self):
        pass


class Mini_HBridge_DC_Motor_PWM(object):
    '''
    Motor controlled with an mini hbridge from the gpio pins on Rpi
    This can be using the L298N as above, but wired differently with only
    two inputs and no enable line.
    https://www.amazon.com/s/ref=nb_sb_noss?url=search-alias%3Dtoys-and-games&field-keywords=Mini+Dual+DC+Motor+H-Bridge+Driver
    https://www.aliexpress.com/item/5-pc-2-DC-Motor-Drive-Module-Reversing-PWM-Speed-Dual-H-Bridge-Stepper-Motor-Mini
    '''
    def __init__(self, pin_forward, pin_backward, freq = 50, max_duty = 90):
        '''
        max_duy is from 0 to 100. I've read 90 is a good max.
        '''
        import RPi.GPIO as GPIO
        self.pin_forward = pin_forward
        self.pin_backward = pin_backward
        self.max_duty = max_duty
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin_forward, GPIO.OUT)
        GPIO.setup(self.pin_backward, GPIO.OUT)
        
        self.pwm_f = GPIO.PWM(self.pin_forward, freq)
        self.pwm_f.start(0)
        self.pwm_b = GPIO.PWM(self.pin_backward, freq)
        self.pwm_b.start(0)

    def run(self, speed):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed is None:
            return
        
        if speed > 1 or speed < -1:
            raise ValueError( "Speed must be between 1(forward) and -1(reverse)")
        
        self.speed = speed
        self.throttle = int(dk.utils.map_range(speed, -1, 1, -self.max_duty, self.max_duty))
        
        if self.throttle > 0:
            self.pwm_f.ChangeDutyCycle(self.throttle)
            self.pwm_b.ChangeDutyCycle(0)
        elif self.throttle < 0:
            self.pwm_f.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(-self.throttle)
        else:
            self.pwm_f.ChangeDutyCycle(0)
            self.pwm_b.ChangeDutyCycle(0)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm_f.ChangeDutyCycle(0)
        self.pwm_b.ChangeDutyCycle(0)
        self.pwm_f.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

    
class RPi_GPIO_Servo(object):
    '''
    Servo controlled from the gpio pins on Rpi
    '''
    def __init__(self, pin, freq = 50, min=5.0, max=7.8):
        import RPi.GPIO as GPIO
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.pin, freq)
        self.pwm.start(0)
        self.min = min
        self.max = max

    def run(self, pulse):
        import RPi.GPIO as GPIO
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        #I've read 90 is a good max
        self.throttle = dk.map_frange(pulse, -1.0, 1.0, self.min, self.max)
        #print(pulse, self.throttle)
        self.pwm.ChangeDutyCycle(self.throttle)


    def shutdown(self):
        import RPi.GPIO as GPIO
        self.pwm.stop()
        GPIO.cleanup()


class ServoBlaster(object):
    '''
    Servo controlled from the gpio pins on Rpi
    This uses a user space service to generate more efficient PWM via DMA control blocks.
    Check readme and install here:
    https://github.com/richardghirst/PiBits/tree/master/ServoBlaster
    cd PiBits/ServoBlaster/user
    make
    sudo ./servod
    will start the daemon and create the needed device file:
    /dev/servoblaster

    to test this from the command line:
    echo P1-16=120 > /dev/servoblaster

    will send 1200us PWM pulse to physical pin 16 on the pi.

    If you want it to start on boot:
    sudo make install
    '''
    def __init__(self, pin):
        self.pin = pin
        self.servoblaster = open('/dev/servoblaster', 'w')
        self.min = min
        self.max = max

    def set_pulse(self, pulse):
        s = 'P1-%d=%d\n' % (self.pin, pulse)
        self.servoblaster.write(s)
        self.servoblaster.flush()

    def run(self, pulse):
        self.set_pulse(pulse)

    def shutdown(self):
        self.run((self.max + self.min) / 2)
        self.servoblaster.close()


class ArduinoFirmata:
    '''
    PWM controller using Arduino board.
    This is particularly useful for boards like Latte Panda with built it Arduino.
    Standard Firmata sketch needs to be loaded on Arduino side.
    Refer to docs/parts/actuators.md for more details
    '''

    def __init__(self, servo_pin = 6, esc_pin = 5):
        from pymata_aio.pymata3 import PyMata3
        self.board = PyMata3()
        self.board.sleep(0.015)
        self.servo_pin = servo_pin
        self.esc_pin = esc_pin
        self.board.servo_config(servo_pin)
        self.board.servo_config(esc_pin)

    def set_pulse(self, pin, angle):
        try:
            self.board.analog_write(pin, int(angle))
        except:
            self.board.analog_write(pin, int(angle))

    def set_servo_pulse(self, angle):
        self.set_pulse(self.servo_pin, int(angle))

    def set_esc_pulse(self, angle):
        self.set_pulse(self.esc_pin, int(angle))



class ArdPWMSteering:
    """
    Wrapper over a Arduino Firmata controller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self,
                 controller=None,
                 left_pulse=60,
                 right_pulse=120):

        self.controller = controller
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.pulse = dk.utils.map_range(0, self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)
        self.running = True
        print('Arduino PWM Steering created')

    def run(self, angle):
        # map absolute angle to angle that vehicle can implement.
        self.pulse = dk.utils.map_range(angle,
                                        self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)
        self.controller.set_servo_pulse(self.pulse)

    def shutdown(self):
        # set steering straight
        self.pulse = dk.utils.map_range(0, self.LEFT_ANGLE, self.RIGHT_ANGLE,
                                        self.left_pulse, self.right_pulse)
        time.sleep(0.3)
        self.running = False


class ArdPWMThrottle:

    """
    Wrapper over Arduino Firmata controller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE = 1

    def __init__(self,
                 controller=None,
                 max_pulse=105,
                 min_pulse=75,
                 zero_pulse=90):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse
        self.pulse = zero_pulse

        # send zero pulse to calibrate ESC
        print("Init ESC")
        self.controller.set_esc_pulse(self.max_pulse)
        time.sleep(0.01)
        self.controller.set_esc_pulse(self.min_pulse)
        time.sleep(0.01)
        self.controller.set_esc_pulse(self.zero_pulse)
        time.sleep(1)
        self.running = True
        print('Arduino PWM Throttle created')

    def run(self, throttle):
        if throttle > 0:
            self.pulse = dk.utils.map_range(throttle, 0, self.MAX_THROTTLE,
                                            self.zero_pulse, self.max_pulse)
        else:
            self.pulse = dk.utils.map_range(throttle, self.MIN_THROTTLE, 0,
                                            self.min_pulse, self.zero_pulse)
        self.controller.set_esc_pulse(self.pulse)

    def shutdown(self):
        # stop vehicle
        self.run(0)
        self.running = False
