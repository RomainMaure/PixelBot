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
        
        # # Read the story script
        # story_script_path = get_package_share_directory('pixelbot_interaction') + "/story/story_script.txt"
        # with open(story_script_path, 'r') as story_script:
        #     self.story = story_script.readlines()

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
        _ = self.send_speak_request("Bonjour, je suis un robot et je m’appelle PixelBote.")

        # Hand waving greeting gesture
        _ = self.send_hand_waving_request()

        # Sentence 1, 2
        _ = self.send_speak_request("Aujourd’hui, je vais vous raconter une histoire à propos de moi et de la mission qui m’a été confiée. J’espère que vous pourrez m’aider!")
        _ = self.send_speak_request("Un jour, un juge d’une autre ville est venu à l’EPFL et a dit aux ingénieurs qu’il avait besoin d’un robot pour une mission délicate.")

        # Show judge location
        _ = self.send_display_location_request("judge")

        # Sentence 3, 4, 5
        _ = self.send_speak_request("Cette mission est de découvrir s'il y a des inégalités entre les hommes et les femmes dans sa ville.")
        _ = self.send_speak_request("Est-ce que vous savez ce qu’est une inégalité entre hommes et femmes? Je vous laisse un peu de temps pour que vous puissiez discuter entre vous.")
        _ = self.send_speak_request("Si votre réponse est oui, alors vous pouvez appuyer sur le bouton vert. Si votre réponse est non, alors vous pouvez appuyer sur le bouton bleu.")

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 6
        _ = self.send_speak_request("Très bien, je reprends mon histoire. Une fois arrivé en ville, je me suis d’abord rendu chez des amis: Hugo et Alice.")

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show flat location
        _ = self.send_display_location_request("flat")

        # Sentence 7
        _ = self.send_speak_request("Quand je suis arrivé, j’ai été très surpris car Alice faisait toutes les tâches ménagères alors que Hugo ne faisait rien.")

        # Surprise emotion
        _ = self.send_display_emotion_request("surprise")
        _ = self.send_emotion_antennae_movement_request("surprise")

        # Sentence 8, 9, 10
        _ = self.send_speak_request("Que pensez-vous de cette situation? Cette situation vous semble-t-elle juste ou non? Je vous laisse un peu de temps pour que vous puissiez en discuter entre vous.") 
        _ = self.send_speak_request("Si cette situation vous semble être juste, alors vous pouvez appuyer sur le bouton vert.")
        _ = self.send_speak_request("Si vous pensez que cette situation n’est pas juste et que Hugo devrait aussi faire des tâches ménagères, alors vous pouvez appuyer sur le bouton bleu.") 

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()       
   
        # Sentence 11, 12
        _ = self.send_speak_request("Votre réponse est très intéressante! Laissez-moi vous raconter ce qui s’est passé ensuite.")
        _ = self.send_speak_request("Après ma visite chez Hugo et Alice, j’ai décidé de partir à la station de ski la plus proche.")

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show ski station location
        _ = self.send_display_location_request("ski")

        # Sentence 13, 14, 15, 16, 17, 18, 19
        _ = self.send_speak_request("Là-bas, j’ai pu observer le métier des sauveteurs. Lorsqu’une personne est blessée en haut d’une montagne, les sauveteurs vont la secourir.")
        _ = self.send_speak_request("Comme je suis un robot, je suis très bon en maths. Le directeur de la station de ski m’a donc demandé de l’aider à calculer le salaire de tous les sauveteurs.") 
        _ = self.send_speak_request("Lorsque nous avons comparé le salaire des sauveteurs avec celui que j’ai calculé, nous avons constaté quelque chose de très étrange: les hommes étaient mieux payés que les femmes alors que les femmes et les hommes faisaient exactement le même travail!") 
        _ = self.send_speak_request("Lorsque le directeur a appris cela, il est devenu furieux.") 
        _ = self.send_speak_request("Que pensez-vous de cette situation? Cette situation vous semble-t-elle juste ou non? Je vous laisse un peu de temps pour que vous puissiez en discuter entre vous.") 
        _ = self.send_speak_request("Si vous trouvez cela juste que les hommes soient mieux payés que les femmes alors que leur travail est le même, vous pouvez appuyer sur le bouton vert.") 
        _ = self.send_speak_request("Si vous trouvez que ce n’est pas juste et que, pour un même travail, le salaire des femmes doit être égal à celui des hommes, alors vous pouvez appuyer sur le bouton bleu.") 

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed() 

        # Sentence 20, 21
        _ = self.send_speak_request("Votre réponse est très intéressante! Laissez-moi vous raconter ce qui s’est passé ensuite.")
        _ = self.send_speak_request("Après mon passage à la station de ski, j’ai découvert qu’une agence spatiale, affiliée à la NASA, se trouvait près de la ville. Je suis donc allé la visiter.")

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show space agency location
        _ = self.send_display_location_request("nasa")   

        # Sentence 22, 23, 24
        _ = self.send_speak_request("Alors que j’allais entrer dans le bâtiment, j’ai remarqué une jeune femme assise sur un banc et avec l’air triste. Cette femme s’appelle Marie. Marie est une scientifique qui aimerait devenir astronaute pour aller sur la lune!")
        _ = self.send_speak_request("Elle est donc allée voir la directrice de l’agence spatiale pour lui demander si elle pouvait devenir astronaute. Mais celle-ci lui a dit qu’elle ne pouvait pas devenir astronaute car c’est une fille!")
        _ = self.send_speak_request("J'étais très fâché car Marie est une scientifique très intelligente et qualifiée, elle a donc toutes les qualités requises pour devenir astronaute!")

        # Angry emotion
        _ = self.send_display_emotion_request("angry")
        _ = self.send_emotion_antennae_movement_request("angry")  

        # Sentence 25, 26, 27 
        _ = self.send_speak_request("Et vous, que pensez-vous de cette situation? Cette situation vous semble-t-elle juste ou non? Je vous laisse un peu de temps pour que vous puissiez en discuter entre vous.")
        _ = self.send_speak_request("Si vous trouvez que c’est juste que Marie ne puisse pas devenir astronaute parce que c’est une fille, alors vous pouvez appuyer sur le bouton vert.")
        _ = self.send_speak_request("Si vous trouvez que ce n’est pas juste, alors vous pouvez appuyer sur le bouton bleu.")

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 28, 29
        _ = self.send_speak_request("Vous connaissez maintenant toutes les personnes que j’ai rencontrées pendant ma mission.")
        _ = self.send_speak_request("Le problème est que je ne comprends toujours pas très bien les humains, vous êtes si compliqués! J’ai besoin de votre aide pour savoir ce que je dois dire au juge.")

        # Walking movement
        _ = self.send_walking_movement_request()

        # Show judge location
        _ = self.send_display_location_request("judge")

        # Sentence 30, 31, 32
        _ = self.send_speak_request("Est-ce que vous pensez que les cas que nous avons vu précédemment sont des cas d’inégalité entre hommes et femmes?")
        _ = self.send_speak_request("Si oui, vous pouvez appuyer sur le bouton vert. Si non, vous pouvez appuyer sur le bouton bleu.")
        _ = self.send_speak_request("Je vous laisse un peu de temps pour que vous puissiez en discuter entre vous.")

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()   

        # Sentence 33, 34, 35
        _ = self.send_speak_request("Très bien, y a-t-il un autre cas d'inégalité entre hommes et femmes qui vous vient à l'esprit et que je pourrais ajouter à mon rapport pour le juge?")
        _ = self.send_speak_request("Si oui, vous pouvez me l’expliquer et appuyer sur le bouton vert. Si non, vous pouvez appuyer sur le bouton bleu.")
        _ = self.send_speak_request("Je vous laisse un peu de temps pour que vous puissiez en discuter entre vous.")

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 36, 37
        _ = self.send_speak_request("D’accord, merci pour votre réponse. Dans ma mission, je dois aussi trouver des solutions aux inégalités.")
        _ = self.send_speak_request("Est-ce que vous vous souvenez du cas d’Hugo et Alice, qui avaient une répartition inégale des tâches ménagères? Que feriez-vous, si vous étiez Hugo et Alice, pour rendre la situation égale?")
           
        # Show flat location
        _ = self.send_display_location_request("flat") 

        # Sentence 38, 39
        _ = self.send_speak_request("Je vous laisse un peu de temps pour que vous puissiez en discuter entre vous.")
        _ = self.send_speak_request("Une fois que vous avez trouvé une solution, vous pouvez me l’expliquer puis appuyer sur n’importe quel bouton.")

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()
        
        # Sentence 40
        _ = self.send_speak_request("Votre réponse est très intéressante! Maintenant, est-ce que vous vous souvenez du cas de la station de ski, dans laquelle il y avait une inégalité de salaire entre les femmes et les hommes?")

        # Show ski station location
        _ = self.send_display_location_request("ski") 

        # Sentence 41, 42, 43
        _ = self.send_speak_request("Que feriez-vous si vous étiez à la place du directeur de la station de ski pour résoudre cette inégalité?")
        _ = self.send_speak_request("Je vous laisse un peu de temps pour que vous puissiez en discuter entre vous.")
        _ = self.send_speak_request("Une fois que vous avez trouvé une solution, vous pouvez me l’expliquer puis appuyer sur n’importe quel bouton.")


        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 44
        _ = self.send_speak_request("D’accord, merci pour votre réponse. Pour finir, est-ce que vous vous souvenez du cas de Marie qui a subi une inégalité d’opportunité professionnelle parce que c’est une fille?")

        # Show space agency location
        _ = self.send_display_location_request("nasa")

        # Sentence 45, 46, 47
        _ = self.send_speak_request("Que feriez-vous si vous étiez à la place de Marie pour résoudre cette inégalité?")
        _ = self.send_speak_request("Je vous laisse un peu de temps pour que vous puissiez en discuter entre vous.")
        _ = self.send_speak_request("Une fois que vous avez trouvé une solution, vous pouvez me l’expliquer puis appuyer sur n’importe quel bouton.")

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed() 

        # Sentence 48
        _ = self.send_speak_request("Merci beaucoup pour vos suggestions ! Le juge va les utiliser pour réduire les inégalités entre les hommes et les femmes. Je suis très heureux!")

        # Happy emotion
        _ = self.send_display_emotion_request("happy")
        _ = self.send_emotion_antennae_movement_request("happy") 

        # Sentence 49, 50, 51
        _ = self.send_speak_request("C’est la fin de mon histoire, merci de m’avoir écouté. Est-ce que vous avez l’impression de mieux comprendre ce que sont les inégalités entre hommes et femmes maintenant?")
        _ = self.send_speak_request("Je vous laisse un peu de temps pour en discuter entre vous.")
        _ = self.send_speak_request("Si votre réponse est oui, alors vous pouvez appuyer sur le bouton vert. Si votre réponse est non, alors vous pouvez appuyer sur le bouton bleu.")

        # Wait for button state change
        self.wait_for_buttons_to_be_pressed()

        # Sentence 52
        _ = self.send_speak_request("Au revoir!")      


def main(args=None):
    rclpy.init(args=args)

    interaction_node = Interaction()

    interaction_node.interaction()

    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
