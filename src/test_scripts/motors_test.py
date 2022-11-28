from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

while True:
    angle = input("enter: ")
    kit.servo[0].angle = int(angle)
    kit.servo[1].angle = int(angle)
    kit.servo[2].angle = int(angle)
    kit.servo[3].angle = int(angle)
