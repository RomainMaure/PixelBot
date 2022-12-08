import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty

from pixelbot_msgs.srv import DisplayEmotion
from pixelbot_msgs.srv import DisplayLocation
from pixelbot_msgs.srv import SetSpeech

from gpiozero import Button


class Interaction(Node):

    RIGHT_BUTTON_PIN, LEFT_BUTTON_PIN = 13, 5
    RIGHT_BUTTON, LEFT_BUTTON = 0, 1

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
            if self.right_button.is_pressed:
                button_pressed = self.RIGHT_BUTTON
                break
            if self.left_button.is_pressed:
                button_pressed = self.LEFT_BUTTON
                break

        return button_pressed

    def interaction(self):
        """
        Main interaction.
        """

        # CHECK CALL OR CALL ASYNC and see if need _
        
        # Sentence 0
        _ = self.send_speak_request("Bonjour, je suis un robot et je m’appelle PixelBote.")

        # Hand waving greeting gesture
        _ = self.send_hand_waving_request()

        # Sentence 1, 2
        _ = self.send_speak_request("Aujourd’hui, je vais vous raconter une histoire à propos de moi et de la mission qui m’a été confiée. J’espère que vous pourrez m’aider!")
        _ = self.send_speak_request("Un jour, un juge d’une autre ville est venu à l’EPFL et a dit aux ingénieurs qu’il avait besoin d’un robot pour une mission délicate.")

        # Show judge location
        _ = self.send_display_location_request("judge")

        # Sentence 3, 4
        _ = self.send_speak_request("Cette mission est de découvrir s'il y a des inégalités entre les hommes et les femmes dans sa ville.")
        _ = self.send_speak_request("Une fois arrivé en ville, je me suis d’abord rendu chez des amis: Hugo et Alice.")

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show flat location
        _ = self.send_display_location_request("flat")

        # Sentence 5
        _ = self.send_speak_request("Quand je suis arrivé, j’ai été très surpris car Alice faisait toutes les tâches ménagères alors que Hugo était assis sur le canapé.")

        # Surprise emotion
        _ = self.send_display_emotion_request("surprise")
        _ = self.send_emotion_antennae_movement_request("surprise")

        # Wait for button state change
        button_pressed = self.wait_for_buttons_to_be_pressed()   

        # Sentence 6
        if button_pressed == self.RIGHT_BUTTON:
            _ = self.send_speak_request("Super, je suis donc allé encourager Alice à en discuter avec Hugo et lui demander son aide.")
        elif button_pressed == self.LEFT_BUTTON:
            _ = self.send_speak_request("Super, je suis donc allé demander à Hugo de se lever et d’aller aider Alice.") 

        # Sentence 7
        _ = self.send_speak_request("Merci pour votre aide, Hugo aide désormais Alice à faire les tâches ménagères et Alice est très contente.")

        # Happy emotion
        _ = self.send_display_emotion_request("happy")
        _ = self.send_emotion_antennae_movement_request("happy")
        # congrat song !!!!!!!!!!
   
        # Sentence 8
        _ = self.send_speak_request("Après ma visite chez Hugo et Alice, j’ai décidé de partir à la station de ski la plus proche.")

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show ski station location
        _ = self.send_display_location_request("ski")

        # Sentence 9, 10, 11
        _ = self.send_speak_request("Là-bas, j’ai pu observer le métier des sauveteurs. Lorsqu’une personne est blessée en haut d’une montagne, les sauveteurs vont la secourir.")
        _ = self.send_speak_request("Comme je suis un robot, je suis très bon en maths. Le directeur de la station de ski m’a donc demandé de l’aider à calculer le salaire de tous les sauveteurs.") 
        _ = self.send_speak_request("Lorsque nous avons comparé le salaire des sauveteurs avec celui que j’ai calculé, nous avons constaté quelque chose de très étrange: les hommes étaient mieux payés que les femmes alors que les femmes et les hommes faisaient exactement le même travail!")

        # Wait for button state change
        button_pressed = self.wait_for_buttons_to_be_pressed() 

        # Sentence 12
        if button_pressed == self.RIGHT_BUTTON:
            _ = self.send_speak_request("Super, je suis donc allé voir le directeur pour lui demander d’équilibrer les salaires.")
        elif button_pressed == self.LEFT_BUTTON:
            _ = self.send_speak_request("Super, je suis donc allé en parler à un secouriste homme. Ce secouriste homme est ensuite allé demander au directeur d’équilibrer les salaires.") 

        # Sentence 13
        _ = self.send_speak_request("Merci pour votre aide, les femmes et les hommes travaillant à la station de ski ont tous le même salaire et sont très contents maintenant!")

        # Happy emotion
        _ = self.send_display_emotion_request("happy")
        _ = self.send_emotion_antennae_movement_request("happy")
        # congrat song !!!!!!!!!!

        # Sentence 14
        _ = self.send_speak_request("Après mon passage à la station de ski, j’ai découvert qu’une agence spatiale, affiliée à la NASA, se trouvait près de la ville. Je suis donc allé la visiter.")

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show space agency location
        _ = self.send_display_location_request("nasa")   

        # Sentence 15, 16, 17
        _ = self.send_speak_request("Alors que j’allais entrer dans le bâtiment, j’ai remarqué une jeune femme assise sur un banc et avec l’air triste. Cette femme s’appelle Marie. Marie est une scientifique qui aimerait devenir astronaute pour aller sur la lune!")
        _ = self.send_speak_request("Elle est donc allée voir la directrice de l’agence spatiale pour lui demander si elle pouvait devenir astronaute. Mais celle-ci lui a dit qu’elle ne pouvait pas devenir astronaute car c’est une fille!")
        _ = self.send_speak_request("J'étais très fâché car Marie est une scientifique très intelligente et qualifiée, elle a donc toutes les qualités requises pour devenir astronaute!")

        # Angry emotion
        _ = self.send_display_emotion_request("angry")
        _ = self.send_emotion_antennae_movement_request("angry") 

        # Wait for button state change
        button_pressed = self.wait_for_buttons_to_be_pressed()

        # Sentence 18
        if button_pressed == self.RIGHT_BUTTON:
            _ = self.send_speak_request("Super, je suis donc allé voir Marie et je lui ai conseillé de persévérer jusqu’à ce qu’elle réussisse.")
        elif button_pressed == self.LEFT_BUTTON:
            _ = self.send_speak_request("Super, je suis donc allé voir la directrice pour lui dire qu’elle avait été un modele de femme courageuse pour Marie. La directrice a alors changé d’avis.") 

        # Sentence 19
        _ = self.send_speak_request("Merci pour votre aide, Marie a réussi à devenir astronaute et est très contente maintenant!")

        # Happy emotion
        _ = self.send_display_emotion_request("happy")
        _ = self.send_emotion_antennae_movement_request("happy")
        # congrat song !!!!!!!!!!

        # Sentence 20
        _ = self.send_speak_request("Pour finir, je suis retourné voir le juge pour lui faire un résumé de ce que j’ai découvert à propos des inégalités entre hommes et femmes dans la ville.")

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show judge location
        _ = self.send_display_location_request("judge")

        # Sentence 21, 22
        _ = self.send_speak_request("J’ai raconté au juge toutes nos aventures et les solutions que nous avons trouvées.")
        _ = self.send_speak_request("Le juge vous remercie pour toutes vos suggestions. Il va les utiliser pour réduire les inégalités entre les hommes et les femmes. Je suis très heureux!")

        # Happy emotion
        _ = self.send_display_emotion_request("happy")
        _ = self.send_emotion_antennae_movement_request("happy") 
        # congrat song

        # Sentence 23
        _ = self.send_speak_request("Au revoir!")      


def main(args=None):
    rclpy.init(args=args)

    interaction_node = Interaction()

    interaction_node.interaction()

    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
