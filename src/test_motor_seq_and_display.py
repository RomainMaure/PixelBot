# System imports
import time
import RPi.GPIO as GPIO

# TODO: autoformating scripts
# TODO: StepPin -> step_pin_ ou _step_pin, define, etc
# TODO: use current step to track pos, home function etc

class StepperConfig():
    
    __CLOCKWISE = 1
    __ANTI_CLOCKWISE = 0
    __CHARGE_PUMP_STABILIZATION_TIME = 0.001
    
    def __init__(self, resetPin, sleepPin, stepPin, directionPin, stepsPerRevolution=200):
        # Configure instance
        self.CLOCKWISE = self.__CLOCKWISE
        self.ANTI_CLOCKWISE = self.__ANTI_CLOCKWISE
        self.ResetPin = resetPin
        self.SleepPin = sleepPin
        self.StepPin = stepPin
        self.DirectionPin = directionPin
        self.RevolutionSteps = stepsPerRevolution
        self.CurrentDirection = self.CLOCKWISE
        self.CurrentStep = 0

        # Setup gpio pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.ResetPin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.SleepPin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.StepPin, GPIO.OUT)
        GPIO.setup(self.DirectionPin, GPIO.OUT)

    def enable_sleep_mode(self):
        GPIO.output(self.SleepPin, GPIO.LOW)

    def disable_sleep_mode(self):
        GPIO.output(self.SleepPin, GPIO.HIGH)
        time.sleep(self.__CHARGE_PUMP_STABILIZATION_TIME)

    # TODO: set_direction
    # TODO: set_step

class SteppersHandler():
    
    def __init__(self, motor_config_1, motor_config_2, delay=0.208):
        self.motor_config_1 = motor_config_1
        self.motor_config_2 = motor_config_2
        self.Delay = delay
        
    def Step(self, stepsToTake, direction, motorConfig):
        # Set the direction
        GPIO.output(motorConfig.DirectionPin, direction)
        
        # Take requested number of steps
        for i in range(stepsToTake):
            GPIO.output(motorConfig.StepPin, GPIO.HIGH)
            motorConfig.CurrentStep += 1 # change to indicate position as well
                
            time.sleep(self.Delay)
            
            GPIO.output(motorConfig.StepPin, GPIO.LOW)
                
            time.sleep(self.Delay)            
                

    def Simultaneous_step(self, stepsToTake, direction1, direction2):
        # Set the directions
        GPIO.output(self.motor_config_1.DirectionPin, direction1)
        GPIO.output(self.motor_config_2.DirectionPin, direction2)

        # Take requested number of steps
        for i in range(stepsToTake):
            GPIO.output(self.motor_config_1.StepPin, GPIO.HIGH)
            GPIO.output(self.motor_config_2.StepPin, GPIO.HIGH)
            self.motor_config_1.CurrentStep += 1 # change to indicate position as well
            self.motor_config_2.CurrentStep += 1 # change to indicate position as well
                
            time.sleep(self.Delay)
            
            GPIO.output(self.motor_config_1.StepPin, GPIO.LOW)
            GPIO.output(self.motor_config_2.StepPin, GPIO.LOW)
                
            time.sleep(self.Delay)

    def enable_sleep(self):
        self.motor_config_1.enable_sleep_mode()
        self.motor_config_2.enable_sleep_mode()

    def disable_sleep(self):
        self.motor_config_1.disable_sleep_mode()
        self.motor_config_2.disable_sleep_mode()

# Define pins
RESET_PIN_TILT_MOT = 6
SLEEP_PIN_TILT_MOT = 13
STEP_PIN_TILT_MOT = 19
DIRECTION_PIN_TILT_MOT = 26

RESET_PIN_PAN_MOT = 4
SLEEP_PIN_PAN_MOT = 17
STEP_PIN_PAN_MOT = 27
DIRECTION_PIN_PAN_MOT = 22


pan_motor_config = StepperConfig(RESET_PIN_PAN_MOT, SLEEP_PIN_PAN_MOT, STEP_PIN_PAN_MOT, DIRECTION_PIN_PAN_MOT)
tilt_motor_config = StepperConfig(RESET_PIN_TILT_MOT, SLEEP_PIN_TILT_MOT, STEP_PIN_TILT_MOT, DIRECTION_PIN_TILT_MOT)
steppersHandler = SteppersHandler(pan_motor_config, tilt_motor_config, 0.005)

steppersHandler.disable_sleep()

steppersHandler.Simultaneous_step(50, pan_motor_config.ANTI_CLOCKWISE, tilt_motor_config.ANTI_CLOCKWISE)

# Sleep 3 sec
start_time = time.time()
steppersHandler.enable_sleep()
while time.time() - start_time < 3.0:
    time.sleep(0.1)

# Wake up
steppersHandler.disable_sleep()

steppersHandler.Step(70, pan_motor_config.CLOCKWISE, pan_motor_config)

# Sleep 3 sec
start_time = time.time()
steppersHandler.enable_sleep()
while time.time() - start_time < 3.0:
    time.sleep(0.1)

# Wake up
steppersHandler.disable_sleep()

steppersHandler.Step(70, tilt_motor_config.CLOCKWISE, tilt_motor_config)

steppersHandler.enable_sleep()



