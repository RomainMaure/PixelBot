from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

while True:
    angle = input("enter an angle between 0 and 180: ")
    kit.servo[0].angle = int(angle)
    kit.servo[1].angle = int(angle)
    kit.servo[2].angle = int(angle)
    kit.servo[3].angle = int(angle)
