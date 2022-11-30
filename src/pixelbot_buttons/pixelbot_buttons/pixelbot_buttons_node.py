import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray

from gpiozero import Button


class Buttons(Node):

    # Defines
    RIGHT_BUTTON_PIN = 5
    LEFT_BUTTON_PIN = 13

    def __init__(self):
        super().__init__('pixelbot_buttons_node')

        # Create publisher of the buttons' state
        self.buttons_publisher = self.create_publisher(UInt8MultiArray, 'buttons_state', 10)
        timer_period = 1/30
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Create the button objects
        self.right_button = Button(self.RIGHT_BUTTON_PIN)
        self.left_button = Button(self.LEFT_BUTTON_PIN)

    def timer_callback(self):
        """
        Callback publishing the buttons' state
        """

        msg = UInt8MultiArray()
        msg.data = [int(self.right_button.is_pressed), int(self.left_button.is_pressed)]
        self.buttons_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    buttons_node = Buttons()

    rclpy.spin(buttons_node)

    buttons_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
