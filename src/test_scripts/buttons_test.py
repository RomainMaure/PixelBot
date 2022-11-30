from gpiozero import Button
from time import sleep

right_button = Button(5)
left_button = Button(13)

while True:
    print("Right button: ", right_button.is_pressed, \
          ", Left button: ", left_button.is_pressed)
    sleep(1)
