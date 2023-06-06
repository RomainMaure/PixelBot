import pyttsx3
import numpy as np
import scipy.io.wavfile as sc
from gtts import gTTS
from pydub import AudioSegment
from playsound import playsound


def speak(text, alteredVoice=False):
    """Allow PixelBot to speak.
    :param text: Text to be said.
    :param alteredVoice: If we want PixelBot's voice to sound more
                            robotic like.
    """
    if not alteredVoice:
        engine.say(text)
        engine.runAndWait()
    else:
        # Create an audio file containing the speech to alter
        tts = gTTS(text, lang='fr')
        tts.save('voiceToAlter.mp3')
        sound = AudioSegment.from_mp3('voiceToAlter.mp3')
        sound.export('voiceToAlter.wav', format='wav')

        # Alter the previously created audio file
        outputFileName = diodeRingModulator('voiceToAlter.wav')

        # Play the altered audio file
        sound = AudioSegment.from_wav(outputFileName)
        sound.export("alteredVoice.mp3", format='mp3')
        playsound("alteredVoice.mp3")

def diode(signalArray):
    """Apply an approximation of the diode non linearity model.
    The approximation of the diode is described by:
            { 0           , if x <= 0
    f(x) = |
            { 0.1*x^(1.7) , if x > 0
    :param signalArray: The signal to be altered.
    :return: The signal altered by the diode non linearity.
    """
    diodeArray = [0.1*(x**1.7) if x > 0 else 0.0 for x in signalArray]

    return np.array(diodeArray)

def diodeRingModulator(intputFileName):
    """Simulate a diode ring modulator electrical circuit.
    Alter the audio file containing the text to be said to make it sounds
    more robotic like.
    :param intputFileName: Name of the audio file containing the voice to
                            be altered.
    :return: Name of the altered audio file.
    """
    # Read the audio file
    [_, data] = sc.read(intputFileName)

    # Get maximum absolute value of input signal
    maxVal = max(abs(data))

    # Scale down the input signal
    scaledData = data/maxVal

    # Create carrier signal
    fCarrier = 500
    t = np.linspace(0, len(scaledData), len(scaledData))
    carrier = np.sin(2*np.pi*fCarrier*t)

    # Compute output of the ring modulator circuit
    topFirst = carrier + 0.5*scaledData
    top = diode(topFirst) + diode(-topFirst)

    bottomFirst = carrier - 0.5*scaledData
    bottom = diode(bottomFirst) + diode(-bottomFirst)

    output = top - bottom

    # Scale back
    output = 5*maxVal*output

    # Save the signal
    sc.write('alteredVoice.wav', 22050, np.int16(output))

    return 'alteredVoice.wav'


# With gtts and then modified to sound more robotic like
speak("Bonjour, ceci est un test.", alteredVoice=True)
