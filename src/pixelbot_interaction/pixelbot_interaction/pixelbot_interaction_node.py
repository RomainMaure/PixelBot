import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from std_msgs.msg import UInt8MultiArray, String

from pixelbot_msgs.srv import DisplayEmotion


class Interaction(Node):

    LEFT_BUTTON = 1

    def __init__(self):
        super().__init__('pixelbot_interaction_node')

        # Subscription to the buttons' state
        self.buttons_subscriber = self.create_subscription(UInt8MultiArray, 'buttons_state', self.buttons_callback, 10)

        # Publisher to make PixelBot speak
        self.speak_publisher = self.create_publisher(String, 'tts_input', 10)
        
        # Create client to perform emotion
        self.display_emotion_cli = self.create_client(DisplayEmotion, 'display_emotion')

        # Create client to perform hand waving movement
        self.hand_waving_cli = self.create_client(Empty, 'hand_waving') 

        # Wait for the clients to be ready
        for client in [self.display_emotion_cli, self.hand_waving_cli]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{client.srv_name} service not available, waiting again...')

        self.left_button_previous_state = None

    def send_display_emotion_request(self, desired_emotion):
        """
        Send a request to the display_emotion service server.

        :param desired_emotion: String to specify which emotion
                                should be displayed.
        """

        # Create a DisplayEmotion request
        request = DisplayEmotion.Request()
        request.desired_emotion = desired_emotion

        # Send the request
        self.future = self.display_emotion_cli.call_async(request)

    def send_hand_waving_request(self):
        """
        Send a request to the hand_waving service server.
        """

        # Create request
        request = Empty.Request()

        # Send request
        self.future = self.hand_waving_cli.call_async(request) 

    def buttons_callback(self, msg):
        """
        Callback receiving the buttons state and trigering behavior if
        the left button is pressed
        """

        # Read left button current state
        left_button_current_state = msg.data[self.LEFT_BUTTON]

        # Behavior if left button is pressed
        if self.left_button_previous_state == False and left_button_current_state == 1:
            # Publish the vlm reply so that it is said by the tts node
            tts_msg = String()
            tts_msg.data = "Hi there, my name is Pixelbot, nice to meet you"
            self.speak_publisher.publish(tts_msg)

            self.send_hand_waving_request()
            self.send_display_emotion_request("happy")

        # Update left button state
        self.left_button_previous_state = left_button_current_state

def main(args=None):
    rclpy.init(args=args)

    interaction_node = Interaction()

    rclpy.spin(interaction_node)

    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
