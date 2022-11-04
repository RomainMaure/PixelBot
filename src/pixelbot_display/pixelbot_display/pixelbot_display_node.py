import rclpy
from rclpy.node import Node

from pixelbot_msgs.srv import DisplayEmotion

from ament_index_python import get_package_share_directory

import time
import pygame
from pygame.locals import *


class Display(Node):

    # Defines
    BLINK_NUMBER_IMAGES = 5
    HAPPY_NUMBER_IMAGES = 5
    BLINKING, HAPPY_FORWARD, HAPPY_BACKWARD = 0, 1, 2
    COUNT_CHANGE_IMAGE_OF_SEQUENCE = 2
    COUNT_TRIGGER_BLINKING_SEQUENCE = 180
    EMOTION_STRING_TO_DEFINE = {"happy": HAPPY_FORWARD}

    def __init__(self):
        super().__init__('pixelbot_display_node')

        # Timer callback for the display
        timer_period = 1.0 / 30.0 # 30 fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Service to perform desired emotion
        self.emotion_srv = self.create_service(DisplayEmotion, 'display_emotion', self.emotion_callback)

        # Initialize Pygame
        pygame.init()

        # Create window
        self.window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        # Do not show computer mouse
        pygame.mouse.set_visible(False)

        # Get screen size
        infoObject = pygame.display.Info()
        dimensions = (infoObject.current_w, infoObject.current_h)

        # Load blinking image sequence
        self.blink_sequence = self.load_sequence("blink", self.BLINK_NUMBER_IMAGES, dimensions)
        self.blink_sequence += reversed(self.blink_sequence)
        self.blink_sequence.remove(self.blink_sequence[len(self.blink_sequence)//2])

        # Load happy image sequences
        self.happy_forward_sequence = self.load_sequence("happy", self.HAPPY_NUMBER_IMAGES, dimensions)
        self.happy_backward_sequence = list(reversed(self.happy_forward_sequence))

        # Initial image on window
        self.window.blit(self.blink_sequence[0], (0, 0))

        # General purpose variables
        self.should_display = True
        self.current_emotion = self.BLINKING
        self.previous_emotion = self.BLINKING
        self.next_emotion = None
        self.counter_change_image_of_sequence = 0
        self.emotion_sequence_idx = 0
        self.is_performing_emotion = False
        self.counter_trigger_blinking_sequence = 0
        self.start_time = time.time()

    def load_sequence(self, sequence_name, number_images, dim):
        """
        Allow to load the sequence of images composing a facial animation
        (blinking animation, smilling animation etc). The images are scaled
        to the screen size.

        :param sequence_name: String indicating the facial animation to load.
        :param number_images: Integer representing the number of images in 
                              the desired sequence.
        :param dim: Integer tuple containing the screen height and width.
        :return: List of scaled images composing a facial animation.
        """

        sequence = []
        images_path = get_package_share_directory('pixelbot_display') + "/imgs/"

        for i in range(number_images):
            sequence.append(pygame.image.load(images_path + sequence_name + str(i) + ".png").convert_alpha())

        sequence = [pygame.transform.scale(img, dim) for img in sequence]

        return sequence

    def play_sequence(self, emotion_sequence):
        """
        Do the transition between two images of a sequence.

        :param emotion_sequence: List of images composing an emotion sequence.
        """

        self.counter_change_image_of_sequence += 1

        if self.counter_change_image_of_sequence == self.COUNT_CHANGE_IMAGE_OF_SEQUENCE and \
        self.emotion_sequence_idx < len(emotion_sequence) - 1:

            self.emotion_sequence_idx += 1
            self.counter_change_image_of_sequence = 0

            self.window.blit(emotion_sequence[self.emotion_sequence_idx], (0, 0))

    def stop_pygame(self):
        """
        Stop Pygame.
        """

        pygame.quit()

    def timer_callback(self):
        """
        Main loop of the display.
        """

        # Event catching
        for event in pygame.event.get():
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                self.should_display = False

        # Reset variables for emotion display
        if self.previous_emotion != self.current_emotion:
            self.counter_change_image_of_sequence = 0
            self.emotion_sequence_idx = 0
            self.previous_emotion = self.current_emotion

        # Emotions display 
        if self.current_emotion == self.BLINKING:
            self.counter_trigger_blinking_sequence += 1

            if self.counter_trigger_blinking_sequence == self.COUNT_TRIGGER_BLINKING_SEQUENCE:
                self.is_performing_emotion = True

            if self.is_performing_emotion:
                self.play_sequence(self.blink_sequence)

                if self.emotion_sequence_idx == len(self.blink_sequence) - 1:
                    self.counter_trigger_blinking_sequence = 0
                    self.emotion_sequence_idx = 0
                    self.is_performing_emotion = False

        elif self.current_emotion == self.HAPPY_FORWARD:
            if self.counter_change_image_of_sequence == 0 and self.emotion_sequence_idx == 0:
                self.start_time = time.time()

            self.play_sequence(self.happy_forward_sequence)

            if time.time() - self.start_time > 3:
                self.current_emotion = self.HAPPY_BACKWARD
                # self.counter_change_image_of_sequence = 0
                # self.emotion_sequence_idx = 0

        elif self.current_emotion == self.HAPPY_BACKWARD:
            self.play_sequence(self.happy_backward_sequence)

            if self.emotion_sequence_idx == len(self.happy_backward_sequence) - 1:
                self.current_emotion = self.BLINKING
                self.is_performing_emotion = False

        # Emotion requests handling
        if self.next_emotion is not None and not self.is_performing_emotion:
            self.is_performing_emotion = True
            self.current_emotion = self.next_emotion
            self.next_emotion = None

        pygame.display.flip()

    def emotion_callback(self, request, response):
        """
        Service handler allowing to perform a desired emotion.

        :param request: See DisplayEmotion message definition.
        :param response: See DisplayEmotion message definition.
        """

        if request.desired_emotion in self.EMOTION_STRING_TO_DEFINE.keys():
            self.next_emotion = self.EMOTION_STRING_TO_DEFINE[request.desired_emotion]
            response.success = True
        else:
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)

    display_node = Display()

    while rclpy.ok() and display_node.should_display:
        rclpy.spin_once(display_node)

    display_node.stop_pygame()

    display_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
