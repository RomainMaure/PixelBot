# System imports
import time
import RPi.GPIO as GPIO

# TODO: autoformating scripts
# TODO: StepPin -> step_pin_ ou _step_pin, define, etc

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
                
            time.sleep(self.Delay)
            
            if i < stepsToTake1:
                GPIO.output(self.motor_config_1.StepPin, GPIO.LOW)
            if i < stepsToTake2:
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
steppersHandler = SteppersHandler(pan_motor_config, tilt_motor_config, 0.0025)

# Tilt motor 100 steps forward
#steppersHandler.Step(0, 100, pan_motor_config.CLOCKWISE, tilt_motor_config.ANTI_CLOCKWISE)

# Pan and Tilt motors 100 steps forward
steppersHandler.Step(100, 100, pan_motor_config.CLOCKWISE, tilt_motor_config.CLOCKWISE)

# Sleep 5 sec
start_time = time.time()
steppersHandler.enable_sleep()
while time.time() - start_time < 5.0:
    time.sleep(0.1)

# Wake up
steppersHandler.disable_sleep()

# Go backward once (all motors)
steppersHandler.Step(100, 100, pan_motor_config.ANTI_CLOCKWISE, tilt_motor_config.ANTI_CLOCKWISE)

# Sleep 10 mins
start_time = time.time()
steppersHandler.enable_sleep()
while time.time() - start_time < 10.0*60.0:
    time.sleep(0.1)

# Wake up
steppersHandler.disable_sleep()
print("finished")

# Go forward once (separately)
#steppersHandler.Step(200, 0, stepperConfig1.CLOCKWISE, stepperConfig2.CLOCKWISE)
#steppersHandler.Step(0, 200, stepperConfig1.CLOCKWISE, stepperConfig2.CLOCKWISE)

# Go backward once (separately)
#steppersHandler.Step(200, 0, stepperConfig1.ANTI_CLOCKWISE, stepperConfig2.ANTI_CLOCKWISE)
#steppersHandler.Step(0, 200, stepperConfig1.ANTI_CLOCKWISE, stepperConfig2.ANTI_CLOCKWISE)

