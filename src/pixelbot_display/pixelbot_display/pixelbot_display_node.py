import rclpy
from rclpy.node import Node

from pixelbot_msgs.srv import DisplayEmotion

import time
import pygame
from pygame.locals import *
import numpy as np
import random

from ament_index_python import get_package_share_directory


class Display(Node):

    # DEFINE
    WHITE = (255, 255, 255)

    def __init__(self):
        super().__init__('pixelbot_display_node')

        # Initialize Pygame
        pygame.init()

        # Create Pygame window
        self.window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

        # Do not show computer mouse
        pygame.mouse.set_visible(False)

        # Defines
        self.FPS = 60.0
        self.ANIMATION_INTERVAL = 3
        self.WIDTH, self.HEIGHT = self.window.get_size()
        self.RIGHT_EYE_CENTER_X = 0.25*self.WIDTH
        self.LEFT_EYE_CENTER_X = 0.75*self.WIDTH
        self.EYES_CENTER_Y = 0.5*self.HEIGHT
        self.RIGHT_EYE_CENTER = (self.RIGHT_EYE_CENTER_X, self.EYES_CENTER_Y)
        self.LEFT_EYE_CENTER = (self.LEFT_EYE_CENTER_X, self.EYES_CENTER_Y)
        self.EYE_RADIUS = 0.75*self.HEIGHT
        self.PROCESSING_STATE_INTERVAL = 2

        self.BLINKING_ANIMATION_EYES_HEIGHT = np.concatenate((np.linspace(self.EYE_RADIUS, 0, num=10),
                                                              np.linspace(0, self.EYE_RADIUS, num=10)))

        self.SURPRISED_ANIMATION_EYES_RADIUS = np.concatenate((np.linspace(self.EYE_RADIUS, 0.45*self.HEIGHT, num=10),
                                                               np.linspace(0.45*self.HEIGHT, 0.85*self.HEIGHT, num=10),
                                                               np.full(150, 0.85*self.HEIGHT),
                                                               np.linspace(0.85*self.HEIGHT, self.EYE_RADIUS, num=30)))

        self.SLIDING_ELLIPSE_POSITION = np.concatenate((np.linspace(-0.1*self.HEIGHT, 0.3*self.HEIGHT, num=30),
                                                        np.full(150, 0.3*self.HEIGHT),
                                                        np.linspace(0.3*self.HEIGHT, -0.1*self.HEIGHT, num=30)))

        self.SLIDING_RECT_POSITION = np.concatenate((np.linspace(-0.2*self.HEIGHT, 0.2*self.HEIGHT, num=20),
                                                     np.full(150, 0.2*self.HEIGHT),
                                                     np.linspace(0.2*self.HEIGHT, -0.2*self.HEIGHT, num=30)))

        self.ANIMATION_LENGTHS = {"blinking": len(self.BLINKING_ANIMATION_EYES_HEIGHT), 
                                  "surprise": len(self.SURPRISED_ANIMATION_EYES_RADIUS), 
                                  "happy": len(self.SLIDING_ELLIPSE_POSITION),
                                  "sad": len(self.SLIDING_ELLIPSE_POSITION), 
                                  "angry": len(self.SLIDING_RECT_POSITION)}
        
        self.AVAILABLE_EMOTIONS = ["surprise", "happy", "sad", "angry"]

        # General purpose variables
        self.should_display = True
        self.animation_running = False
        self.current_emotion = "blinking"
        self.desired_emotion = "blinking"
        self.counter_animation_frame = 0
        self.start_time_animation = time.time()

        # Timer callback for the display
        timer_period = 1.0 / self.FPS # 60 fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Service to perform desired emotion
        self.emotion_srv = self.create_service(DisplayEmotion, 'display_emotion', self.emotion_callback)

        # Initial image on window
        self.window.fill("white")
    
        self.right_eye = pygame.Rect(0, 0, 0, 0)
        self.left_eye = pygame.Rect(0, 0, 0, 0)
        
        self.right_eye.size = (self.EYE_RADIUS, self.EYE_RADIUS)
        self.left_eye.size = (self.EYE_RADIUS, self.EYE_RADIUS)
        
        self.right_eye.center = self.RIGHT_EYE_CENTER
        self.left_eye.center = self.LEFT_EYE_CENTER
        
        pygame.draw.ellipse(self.window, "black", self.right_eye)
        pygame.draw.ellipse(self.window, "black", self.left_eye)

        pygame.display.flip()

    def time_until_next_blink_refractory(self, mean_interval_s=4.0, t_min=0.25):
        """
        Returns the time (in seconds) until the next blink.
        Ensures a minimum spacing t_min (physiological refractory period).
        The remaining time is exponential with adjusted mean.
        """

        # Adjust rate so that mean total interval = mean_interval_s
        lam = 1.0 / max((mean_interval_s - t_min), 0.001)

        return t_min + random.expovariate(lam)

    def blinking(self):
        """
        Blinking animation.

        :param window: The Pygame window to draw on.
        :param counter_animation_frame: Counter keeping track of the animation frame.
        :param right_eye: Pygame Rect object representing the right eye.
        :param left_eye: Pygame Rect object representing the left eye.
        """

        self.right_eye.size = self.EYE_RADIUS, self.BLINKING_ANIMATION_EYES_HEIGHT[self.counter_animation_frame]
        self.left_eye.size = self.EYE_RADIUS, self.BLINKING_ANIMATION_EYES_HEIGHT[self.counter_animation_frame]

        self.right_eye.center = self.RIGHT_EYE_CENTER
        self.left_eye.center = self.LEFT_EYE_CENTER

        pygame.draw.ellipse(self.window, "black", self.right_eye)
        pygame.draw.ellipse(self.window, "black", self.left_eye)

    def surprised(self):
        """
        Surprised animation.

        :param window: The Pygame window to draw on.
        :param counter_animation_frame: Counter keeping track of the animation frame.
        :param right_eye: Pygame Rect object representing the right eye.
        :param left_eye: Pygame Rect object representing the left eye.
        """

        self.right_eye.size = self.SURPRISED_ANIMATION_EYES_RADIUS[self.counter_animation_frame], self.SURPRISED_ANIMATION_EYES_RADIUS[self.counter_animation_frame]
        self.left_eye.size = self.SURPRISED_ANIMATION_EYES_RADIUS[self.counter_animation_frame], self.SURPRISED_ANIMATION_EYES_RADIUS[self.counter_animation_frame]

        self.right_eye.center = self.RIGHT_EYE_CENTER
        self.left_eye.center = self.LEFT_EYE_CENTER

        pygame.draw.ellipse(self.window, "black", self.right_eye)
        pygame.draw.ellipse(self.window, "black", self.left_eye)

    def happy_sad(self, sadness=True):
        """
        Happiness/Sadness animation.

        :param window: The Pygame window to draw on.
        :param counter_animation_frame: Counter keeping track of the animation frame.
        :param right_eye: Pygame Rect object representing the right eye.
        :param left_eye: Pygame Rect object representing the left eye.
        :param sadness: Boolean to choose between the sadness or happiness emotion.
        """

        # Draw round eyes
        self.right_eye.size = (self.EYE_RADIUS, self.EYE_RADIUS)
        self.left_eye.size = (self.EYE_RADIUS, self.EYE_RADIUS)

        self.right_eye.center = self.RIGHT_EYE_CENTER
        self.left_eye.center = self.LEFT_EYE_CENTER

        pygame.draw.ellipse(self.window, "black", self.right_eye)
        pygame.draw.ellipse(self.window, "black", self.left_eye)

        # Draw the sliding white ellipse over the round eyes
        right_ellipse = pygame.Rect(0, 0, 0, 0)
        left_ellipse = pygame.Rect(0, 0, 0, 0)

        right_ellipse.size = (0.85*self.HEIGHT, 0.35*self.HEIGHT)
        left_ellipse.size = (0.85*self.HEIGHT, 0.35*self.HEIGHT)
        
        if sadness:
            right_ellipse.center = (self.RIGHT_EYE_CENTER_X, self.SLIDING_ELLIPSE_POSITION[self.counter_animation_frame])
            left_ellipse.center = (self.LEFT_EYE_CENTER_X, self.SLIDING_ELLIPSE_POSITION[self.counter_animation_frame])
        else:
            right_ellipse.center = (self.RIGHT_EYE_CENTER_X, self.HEIGHT - self.SLIDING_ELLIPSE_POSITION[self.counter_animation_frame])
            left_ellipse.center = (self.LEFT_EYE_CENTER_X, self.HEIGHT - self.SLIDING_ELLIPSE_POSITION[self.counter_animation_frame])

        pygame.draw.ellipse(self.window, "white", right_ellipse)
        pygame.draw.ellipse(self.window, "white", left_ellipse)

    def anger(self):
        """
        Anger animation.

        :param window: The Pygame window to draw on.
        :param counter_animation_frame: Counter keeping track of the animation frame.
        :param right_eye: Pygame Rect object representing the right eye.
        :param left_eye: Pygame Rect object representing the left eye.
        """

        # Draw round eyes
        self.right_eye.size = (self.EYE_RADIUS, self.EYE_RADIUS)
        self.left_eye.size = (self.EYE_RADIUS, self.EYE_RADIUS)

        self.right_eye.center = self.RIGHT_EYE_CENTER
        self.left_eye.center = self.LEFT_EYE_CENTER

        pygame.draw.ellipse(self.window, "black", self.right_eye)
        pygame.draw.ellipse(self.window, "black", self.left_eye)

        # Draw the sliding rotated white rectangle
        # Define a rectangular surface, its transparent colorkey and its color
        rect_surface = pygame.Surface((self.HEIGHT, 0.4*self.HEIGHT))
        rect_surface.set_colorkey("black")
        rect_surface.fill("white")

        # Rotate the rectangular surface
        rotation_angle = 20.0
        right_eye_rectangular_surface = pygame.transform.rotate(rect_surface, -rotation_angle)
        left_eye_rectangular_surface = pygame.transform.rotate(rect_surface, rotation_angle)

        # Define the center of the sliding rotated rectangular surfaces
        right_eye_rect = right_eye_rectangular_surface.get_rect()
        left_eye_rect = left_eye_rectangular_surface.get_rect()

        right_eye_rect.center = self.RIGHT_EYE_CENTER_X, self.SLIDING_RECT_POSITION[self.counter_animation_frame]
        left_eye_rect.center = self.LEFT_EYE_CENTER_X, self.SLIDING_RECT_POSITION[self.counter_animation_frame]

        # Draw the sliding rotated rectangular surfaces over the round eyes
        self.window.blit(right_eye_rectangular_surface, right_eye_rect)
        self.window.blit(left_eye_rectangular_surface, left_eye_rect)


    def timer_callback(self):
        """
        Main loop of the display.
        """

        # Event catching
        for event in pygame.event.get():
            if event.type == KEYDOWN and event.key == K_ESCAPE:
                self.should_display = False
    
        # Delay between each emotion animation:
        # Either time based or request based
        if self.animation_running == False and (time.time() - self.start_time_animation > self.ANIMATION_INTERVAL or self.desired_emotion != "blinking"):
            self.ANIMATION_INTERVAL = self.time_until_next_blink_refractory()
            self.animation_running = True
            self.current_emotion = self.desired_emotion
            self.desired_emotion = "blinking"
        
        # Animation drawing
        if self.animation_running:
            # Fill the window with a white background
            self.window.fill("white")
    
            # Draw the eyes with the desired emotion
            if self.current_emotion == "blinking":
                self.blinking()
            elif self.current_emotion == "surprise":
                self.surprised()
            elif self.current_emotion == "happy":
                self.happy_sad(sadness=False)
            elif self.current_emotion == "sad":
                self.happy_sad(sadness=True)
            elif self.current_emotion == "angry":
                self.anger()
    
            # Update the frame counter and check for end of animation
            self.counter_animation_frame += 1
            animation_lenght = self.ANIMATION_LENGTHS[self.current_emotion]
            if self.counter_animation_frame == animation_lenght:
                self.counter_animation_frame = 0
                self.animation_running = False

                # If no emotion request since the last request, go back to blinking
                # If emotion request since the last request, next emotion is the new requested emotion
                self.current_emotion = self.desired_emotion

                self.start_time_animation = time.time()

            pygame.display.flip()

    def emotion_callback(self, request, response):
        """
        Service handler allowing to perform a desired emotion.

        :param request: See DisplayEmotion service definition.
        :param response: See DisplayEmotion service definition.
        """

        if request.desired_emotion in self.AVAILABLE_EMOTIONS:
            self.desired_emotion = request.desired_emotion
            response.success = True
        else:
            self.get_logger().info("Error: requested emotion does not exist!")
            response.success = False

        return response

    def stop_pygame(self):
        """
        Stop Pygame.
        """

        pygame.quit()


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
