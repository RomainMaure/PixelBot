import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray

from pixelbot_msgs.srv import DisplayEmotion
from pixelbot_msgs.srv import DisplayLocation
from pixelbot_msgs.srv import SetSpeech

from ament_index_python import get_package_share_directory


class Interaction(Node):

    RIGHT_BUTTON, LEFT_BUTTON = 0, 1

    def __init__(self):
        super().__init__('pixelbot_interaction_node')

        # Create client to perform emotion
        self.display_emotion_cli = self.create_client(DisplayEmotion, 'display_emotion')

        # Create client to display location
        self.display_location_cli = self.create_client(DisplayLocation, 'display_location')

        # Create client to make PixelBot speak
        self.speak_cli = self.create_client(SetSpeech, 'speak')

        # Wait for the clients to be ready
        for client in [self.display_emotion_cli, self.display_location_cli, self.speak_cli]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{client.srv_name} service not available, waiting again...')

        # Create subscriber to buttons state
        self.right_button_state = 0
        self.left_button_state = 0
        self.buttons_state_subscription = self.create_subscription(UInt8MultiArray,
                                                                   'buttons_state',
                                                                   self.on_buttons_state_update,
                                                                   10)

        # Read the story script
        story_script_path = get_package_share_directory('pixelbot_interaction') + "/story/story_script.txt"
        with open(story_script_path, 'r') as story_script:
            self.story = story_script.readlines()

    def send_display_emotion_request(self, desired_emotion):
        """
        
        """

        self.request = DisplayEmotion.Request()
        self.request.desired_emotion = desired_emotion

        self.future = self.display_emotion_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_display_location_request(self, desired_location):
        """
        
        """

        self.request = DisplayLocation.Request()
        self.request.desired_location = desired_location

        self.future = self.display_location_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

    def send_speak_request(self, message):
        """
        
        """

        self.request = SetSpeech.Request()
        self.request.message = message

        self.future = self.speak_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

    def on_buttons_state_update(self, msg):
        """
        
        """

        self.right_button_state = msg.data[self.RIGHT_BUTTON]
        self.left_button_state = msg.data[self.LEFT_BUTTON]

    def interaction(self):
        """
        
        """
        
        # CHECK CALL OR CALL ASYNC

        for line_counter in range(5):
            _ = self.send_speak_request(self.story[line_counter])

        # arm movement cli request

        _ = self.send_speak_request(self.story[5])
        # arm movement cli request

        _ = self.send_speak_request(self.story[6])
        # wait for button state change

        _ = self.send_speak_request(self.story[7])
        _ = self.send_display_location_request("flat")

        _ = self.send_speak_request(self.story[8])
        _ = self.send_display_emotion_request("surprise")

        for line_counter in range(9, 12):
            _ = self.send_speak_request(self.story[line_counter])

        # wait for button state change    


def main(args=None):
    rclpy.init(args=args)

    interaction_node = Interaction()

    interaction_node.interaction()

    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
