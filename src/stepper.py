# System imports
import RPi.GPIO as GPIO
from time import sleep

class StepperConfig():
    
    __CLOCKWISE = 1
    __ANTI_CLOCKWISE = 0
    
    def __init__(self, stepPin, directionPin, stepsPerRevolution=200):
        # Configure instance
        self.CLOCKWISE = self.__CLOCKWISE
        self.ANTI_CLOCKWISE = self.__ANTI_CLOCKWISE
        self.StepPin = stepPin
        self.DirectionPin = directionPin
        self.RevolutionSteps = stepsPerRevolution
        self.CurrentDirection = self.CLOCKWISE
        self.CurrentStep = 0

        # Setup gpio pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.StepPin, GPIO.OUT)
        GPIO.setup(self.DirectionPin, GPIO.OUT)

class SteppersHandler():
    
    def __init__(self, motor_config_1, motor_config_2, delay=0.208):
        self.motor_config_1 = motor_config_1
        self.motor_config_2 = motor_config_2
        self.Delay = delay

    def Step(self, stepsToTake1, stepsToTake2, direction1, direction2):
        # Set the directions
        GPIO.output(self.motor_config_1.DirectionPin, direction1)
        GPIO.output(self.motor_config_2.DirectionPin, direction2)

        # Take requested number of steps
        for i in range(max(stepsToTake1, stepsToTake2)):
            if i < stepsToTake1:
                GPIO.output(self.motor_config_1.StepPin, GPIO.HIGH)
                self.motor_config_1.CurrentStep += 1 # change to indicate position as well
            if i < stepsToTake2:
                GPIO.output(self.motor_config_2.StepPin, GPIO.HIGH)
                self.motor_config_2.CurrentStep += 1 # change to indicate position as well
                
            sleep(self.Delay)
            
            if i < stepsToTake1:
                GPIO.output(self.motor_config_1.StepPin, GPIO.LOW)
            if i < stepsToTake2:
                GPIO.output(self.motor_config_2.StepPin, GPIO.LOW)
                
            sleep(self.Delay)

# Define pins
STEP_PIN_TILT_MOT = 16
DIRECTION_PIN_TILT_MOT = 21

STEP_PIN_PAN_MOT = 13
DIRECTION_PIN_PAN_MOT = 26


pan_motor_config = StepperConfig(STEP_PIN_PAN_MOT, DIRECTION_PIN_PAN_MOT)
tilt_motor_config = StepperConfig(STEP_PIN_TILT_MOT, DIRECTION_PIN_TILT_MOT)
steppersHandler = SteppersHandler(pan_motor_config, tilt_motor_config, 0.0025)

# Tilt motor 100 steps forward
#steppersHandler.Step(0, 100, pan_motor_config.CLOCKWISE, tilt_motor_config.ANTI_CLOCKWISE)

# Pan and Tilt motors 100 steps forward
steppersHandler.Step(100, 100, pan_motor_config.ANTI_CLOCKWISE, tilt_motor_config.ANTI_CLOCKWISE)

# Go forward once (all motors)
#steppersHandler.Step(200, 200, stepperConfig1.CLOCKWISE, stepperConfig2.CLOCKWISE)

# Go backward once (all motors)
#steppersHandler.Step(200, 200, stepperConfig1.ANTI_CLOCKWISE, stepperConfig2.ANTI_CLOCKWISE)

# Go forward once (separately)
#steppersHandler.Step(200, 0, stepperConfig1.CLOCKWISE, stepperConfig2.CLOCKWISE)
#steppersHandler.Step(0, 200, stepperConfig1.CLOCKWISE, stepperConfig2.CLOCKWISE)

# Go backward once (separately)
#steppersHandler.Step(200, 0, stepperConfig1.ANTI_CLOCKWISE, stepperConfig2.ANTI_CLOCKWISE)
#steppersHandler.Step(0, 200, stepperConfig1.ANTI_CLOCKWISE, stepperConfig2.ANTI_CLOCKWISE)

