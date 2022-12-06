import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

from pixelbot_msgs.srv import DisplayEmotion
from pixelbot_msgs.srv import DisplayLocation
from pixelbot_msgs.srv import SetSpeech

from ament_index_python import get_package_share_directory

from gpiozero import Button


class Interaction(Node):

    RIGHT_BUTTON_PIN, LEFT_BUTTON_PIN = 13, 5

    def __init__(self):
        super().__init__('pixelbot_interaction_node')

        # Create client to perform emotion
        self.display_emotion_cli = self.create_client(DisplayEmotion, 'display_emotion')

        # Create client to display location
        self.display_location_cli = self.create_client(DisplayLocation, 'display_location')

        # Create client to make PixelBot speak
        self.speak_cli = self.create_client(SetSpeech, 'speak')

        # Create client to perform arm walking movement
        self.walking_movement_cli = self.create_client(Empty, 'walking_movement')

        # Create client to perform hand waving movement
        self.hand_waving_cli = self.create_client(Empty, 'hand_waving') 

        # Create client to perform antennae movements for emotions
        self.emotion_antennae_movement_cli = self.create_client(DisplayEmotion, 'emotion_antennae_movement')

        # Wait for the clients to be ready
        for client in [self.display_emotion_cli, self.display_location_cli, \
                       self.speak_cli, self.walking_movement_cli, \
                       self.hand_waving_cli, self.emotion_antennae_movement_cli]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'{client.srv_name} service not available, waiting again...')

        # Button objects
        self.right_button = Button(self.RIGHT_BUTTON_PIN)
        self.left_button = Button(self.LEFT_BUTTON_PIN)
        
        # Read the story script
        story_script_path = get_package_share_directory('pixelbot_interaction') + "/story/story_script.txt"
        with open(story_script_path, 'r') as story_script:
            self.story = story_script.readlines()

    def send_display_emotion_request(self, desired_emotion):
        """
        Send a request to the display_emotion service server.

        :param desired_emotion: String to specify which emotion
                                should be displayed.
        """

        self.request = DisplayEmotion.Request()
        self.request.desired_emotion = desired_emotion

        self.future = self.display_emotion_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_display_location_request(self, desired_location):
        """
        Send a request to the display_location service server.

        :param desired_location: String to specify which location
                                 should be displayed.
        """

        self.request = DisplayLocation.Request()
        self.request.desired_location = desired_location

        self.future = self.display_location_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

    def send_speak_request(self, message):
        """
        Send a request to the speak service server.

        :param message: String to specify the text to be spoken by PixelBot.
        """

        self.request = SetSpeech.Request()
        self.request.message = message

        self.future = self.speak_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

    def send_walking_movement_request(self):
        """
        Send a request to the walking_movement service server.
        """

        self.request = Empty.Request()

        self.future = self.walking_movement_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_hand_waving_request(self):
        """
        Send a request to the hand_waving service server.
        """

        self.request = Empty.Request()

        self.future = self.hand_waving_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def send_emotion_antennae_movement_request(self, desired_emotion):
        """
        Send a request to the emotion_antennae_movement service server.

        :param desired_emotion: String to specify which emotion
                                should be performed.
        """

        self.request = DisplayEmotion.Request()
        self.request.desired_emotion = desired_emotion

        self.future = self.emotion_antennae_movement_cli.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()

    def wait_for_buttons_to_be_pressed(self):
        """
        Infinite loop waiting for one of the buttons to be pressed.
        """

        while rclpy.ok():
            if self.right_button.is_pressed or self.left_button.is_pressed:
                break

    def interaction(self):
        """
        Main interaction.
        """

        # CHECK CALL OR CALL ASYNC and see if need _
        
        # Sentence 0
        _ = self.send_speak_request(self.story[0])

        # Hand waving greeting gesture
        _ = self.send_hand_waving_request()

        # Sentence 1, 2
        _ = self.send_speak_request(self.story[1])
        _ = self.send_speak_request(self.story[2])

        # Show judge location
        _ = self.send_display_location_request("judge")

        # Sentence 3, 4, 5
        for line_counter in range(3, 6):
            _ = self.send_speak_request(self.story[line_counter])

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 6
        _ = self.send_speak_request(self.story[6])

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show flat location
        _ = self.send_display_location_request("flat")

        # Sentence 7
        _ = self.send_speak_request(self.story[7])

        # Surprise emotion
        _ = self.send_display_emotion_request("surprise")
        _ = self.send_emotion_antennae_movement_request("surprise")

        # Sentence 8, 9, 10
        for line_counter in range(8, 11):
            _ = self.send_speak_request(self.story[line_counter])  

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()       
   
        # Sentence 11, 12
        _ = self.send_speak_request(self.story[11])
        _ = self.send_speak_request(self.story[12])

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show ski station location
        _ = self.send_display_location_request("ski")

        # Sentence 13, 14, 15, 16, 17, 18, 19
        for line_counter in range(13, 20):
            _ = self.send_speak_request(self.story[line_counter]) 

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed() 

        # Sentence 20, 21
        _ = self.send_speak_request(self.story[20])
        _ = self.send_speak_request(self.story[21])

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show space agency location
        _ = self.send_display_location_request("nasa")   

        # Sentence 22, 23, 24
        for line_counter in range(22, 25):
            _ = self.send_speak_request(self.story[line_counter])

        # Angry emotion
        _ = self.send_display_emotion_request("angry")
        _ = self.send_emotion_antennae_movement_request("angry")  

        # Sentence 25, 26, 27 
        for line_counter in range(25, 28):
            _ = self.send_speak_request(self.story[line_counter])

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 28, 29
        for line_counter in range(28, 30):
            _ = self.send_speak_request(self.story[line_counter])

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show judge location
        _ = self.send_display_location_request("judge")

        # Sentence 30, 31, 32
        for line_counter in range(30, 33):
            _ = self.send_speak_request(self.story[line_counter])

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()   

        # Sentence 33, 34, 35
        for line_counter in range(33, 36):
            _ = self.send_speak_request(self.story[line_counter])

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 36, 37
        _ = self.send_speak_request(self.story[36])
        _ = self.send_speak_request(self.story[37])
           
        # Show flat location
        _ = self.send_display_location_request("flat") 

        # Sentence 38, 39
        _ = self.send_speak_request(self.story[38])
        _ = self.send_speak_request(self.story[39])

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()
        
        # Sentence 40
        _ = self.send_speak_request(self.story[40])

        # Show ski station location
        _ = self.send_display_location_request("ski") 

        # Sentence 41, 42, 43
        for line_counter in range(41, 44):
            _ = self.send_speak_request(self.story[line_counter])

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 44
        _ = self.send_speak_request(self.story[44])

        # Show space agency location
        _ = self.send_display_location_request("nasa")

        # Sentence 45, 46, 47
        for line_counter in range(45, 48):
            _ = self.send_speak_request(self.story[line_counter])

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed() 

        # Sentence 48
        _ = self.send_speak_request(self.story[48])

        # Happy emotion
        _ = self.send_display_emotion_request("happy")
        _ = self.send_emotion_antennae_movement_request("happy") 

        # Sentence 49, 50, 51
        for line_counter in range(49, 52):
            _ = self.send_speak_request(self.story[line_counter]) 

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 52
        _ = self.send_speak_request(self.story[52])      


def main(args=None):
    rclpy.init(args=args)

    interaction_node = Interaction()

    interaction_node.interaction()

    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
