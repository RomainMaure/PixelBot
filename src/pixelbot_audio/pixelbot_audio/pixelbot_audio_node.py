import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String

from pixelbot_msgs.srv import SetLanguage, PlaySound

from piper import PiperVoice, SynthesisConfig
import sounddevice as sd
from playsound import playsound
import numpy as np

from ament_index_python import get_package_share_directory


class TTS_Playsound(Node):

    def __init__(self):
        super().__init__('pixelbot_audio_node')

        # Subscription to verbalize what is said on the tts_input topic
        self.speak_subscriber = self.create_subscription(String, 'tts_input', self.topic_speak_callback, 10)

        # Publisher to enable speech recognition after speaking is finished
        self.recognizer_state_publisher = self.create_publisher(Bool, 'recognizer_is_active', 10)

        # Service allowing to change the language to be spoken
        self.language_srv = self.create_service(SetLanguage, 'change_language', self.change_language_callback)

        # Service allowing PixelBot to play a desired sound file
        self.play_sound_srv = self.create_service(PlaySound, 'play_sound', self.play_sound_callback)

        # Load Piper voice
        self.english_voice_path = get_package_share_directory('pixelbot_audio') + "/voices/en_US-bryce-medium.onnx"
        self.french_voice_path = get_package_share_directory('pixelbot_audio') + "/voices/fr_FR-upmc-medium.onnx"
        self.german_voice_path = get_package_share_directory('pixelbot_audio') + "/voices/de_DE-thorsten_emotional-medium.onnx"
        self.voice = PiperVoice.load(self.english_voice_path)
        self.syn_config = SynthesisConfig(volume=1.0,       # default volume
                                     length_scale=1.0,      # normal pace
                                     noise_scale=0.667,     # default audio variation
                                     noise_w_scale=0.8,     # default speaking variation
                                     normalize_audio=False, # use raw audio from voice
                                     )

    def speak(self, message):
        '''TODO:documentation'''

        stream = None

        try:
            for chunk in self.voice.synthesize(message, syn_config=self.syn_config):
                if stream is None:
                    stream = sd.OutputStream(
                        samplerate=chunk.sample_rate,
                        channels=chunk.sample_channels,
                        dtype='int16'
                    )
                    stream.start()

                # Convert the raw bytes to a NumPy array for sounddevice
                audio_array = np.frombuffer(chunk.audio_int16_bytes, dtype=np.int16)
                stream.write(audio_array)

        finally:
            if stream:
                stream.stop()
                stream.close()

    def topic_speak_callback(self, msg):
        '''TODO:documentation'''

        # Deactivate speech recognizer
        recognizer_state_msg = Bool()
        recognizer_state_msg.data = False
        self.recognizer_state_publisher.publish(recognizer_state_msg)

        self.speak(msg.data)

        # Reactivate speech recognizer
        recognizer_state_msg = Bool()
        recognizer_state_msg.data = True
        self.recognizer_state_publisher.publish(recognizer_state_msg)

    def change_language_callback(self, request, response):
        """
        Service handler allowing to change the language to be spoken.

        :param request: See SetLanguage message definition.
        :param response: See SetLanguage service definition.
        """

        if request.language not in ["french", "english", "german"]:
            response.success = False
            self.get_logger().info("Requested language not available.")
        else:
            response.success = True
            if request.language == "french":
                self.voice = PiperVoice.load(self.french_voice_path)
            if request.language == "english":
                self.voice = PiperVoice.load(self.english_voice_path)
            if request.language == "german":
                self.voice = PiperVoice.load(self.german_voice_path)

        return response

    def play_sound_callback(self, request):
        """
        Service handler allowing to play a sound.

        :param request: See PlaySound message definition.
        """

        sound_path = get_package_share_directory('pixelbot_audio') + "/sounds/" + request.sound_name
        playsound(sound_path)


def main(args=None):
    rclpy.init(args=args)

    tts_playsound_node = TTS_Playsound()

    rclpy.spin(tts_playsound_node)

    tts_playsound_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
