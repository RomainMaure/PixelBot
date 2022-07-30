# pip install SpeechRecognition

import speech_recognition as sr
import time

# Initialize the microphone and the recognizer objects
microphone = sr.Microphone()
recognizer = sr.Recognizer()

# Calibrate the recognizer to ambient noise.
with microphone as source:
    # listen for 5 second to calibrate the energy threshold for ambient noise levels
    print("Calibrating: please do not speak")
    recognizer.adjust_for_ambient_noise(source, 5)
    print("Calibrating done")

# Try Recognize the incomming speech.
# :return: The recognized text if the recognition worked or an empty string otherwise.
with microphone as source:
    print("Say something")

    audio = recognizer.listen(source)
    said = ""

    try:
        said = recognizer.recognize_google(audio)
        print(said)
    except:
        print("Sorry, I haven't understand you properly, \
                would you mind to speak louder/slower/closer ?")


def speechRecognitionCallback(recognizer, audio):
    """Recognize the received audio data and update detectedSentence.
    The callback called when we receive audio data (when the interlocutor
    stopped to speak).
    """

    try:
        detectedSentence = recognizer.recognize_google(audio)
    except:
        detectedSentence = ""

    print(detectedSentence)

print("Start listening in background")
stop_listening = recognizer.listen_in_background(microphone, speechRecognitionCallback)

time.sleep(10)

stop_listening(wait_for_stop=True)
