################################# IMPORTS #####################################

import pygame
from pygame.locals import *

import time

################################## FUNCTIONS ##################################

def load_sequence(sequence_name, number_images, dim):
    sequence = []

    for i in range(number_images):
        sequence.append(pygame.image.load("../img/" + sequence_name + "/" \
            + sequence_name + str(i) + ".png").convert_alpha())

    sequence = [pygame.transform.scale(img, dim) for img in sequence]

    return sequence

def play_sequence(emotion_sequence, count_emotion_state, emotion_state):
    
    count_emotion_state += 1

    if count_emotion_state == CHANGE_STATE_TIME and \
       emotion_state < len(emotion_sequence) - 1:

        emotion_state += 1
        count_emotion_state = 0

        window.blit(emotion_sequence[emotion_state], (0, 0))

    return count_emotion_state, emotion_state

################################## VARIABLES ##################################

# Emotions
BLINKING = 0
HAPPY1 = 1
HAPPY2 = 2
CHANGE_STATE_TIME = 5

emotion = BLINKING
previous_emotion = BLINKING
count_emotion_state = 0
emotion_state = 0

# Eye blinking sequence
BLINKING_TIME = 400
BLINK_NUMBER_IMAGES = 5

is_blinking = False
count_blinking = 0

# Happy sequence
HAPPY_NUMBER_IMAGES = 5

# General use
FPS = 100

mainLoop = True

################################## MAIN #######################################

pygame.init()

infoObject = pygame.display.Info()
dimensions = (infoObject.current_w, infoObject.current_h)

# 1920, 1080
window = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)

pygame.mouse.set_visible(False)

blink = load_sequence("blink", BLINK_NUMBER_IMAGES, dimensions)
blink += reversed(blink)
blink.remove(blink[len(blink)//2])


happy1 = load_sequence("happy", HAPPY_NUMBER_IMAGES, dimensions)
happy2 = list(reversed(happy1))

window.blit(blink[0], (0, 0))

clock = pygame.time.Clock()

###
start_time = time.time()

while mainLoop: 

    clock.tick(FPS)

    # Event catching
    for event in pygame.event.get():
        if event.type == KEYDOWN and event.key == K_ESCAPE:
            mainLoop = False

    # Reset variables for emotion display
    if previous_emotion != emotion:
        count_emotion_state = 0
        emotion_state = 0
        previous_emotion = emotion

    # Emotions display 
    # ATTENDRE ANIMATION FINI POUR FAIRE UN AUTRE
    if emotion == BLINKING:
        count_blinking += 1

        if count_blinking == BLINKING_TIME:
            is_blinking = True

        if is_blinking:
            count_emotion_state, emotion_state = play_sequence(blink, 
                                                        count_emotion_state,
                                                        emotion_state)

            if emotion_state == len(blink) - 1:
                count_blinking = 0
                emotion_state = 0
                is_blinking = False

    elif emotion == HAPPY1:
        ###
        if count_emotion_state == 0 and emotion_state == 0:
            start_time = time.time()
        ###

        count_emotion_state, emotion_state = play_sequence(happy1, 
                                                        count_emotion_state,
                                                        emotion_state)

        ###
        if time.time() - start_time > 3:
            emotion = HAPPY2
            count_emotion_state = 0
            emotion_state = 0
        ###

    elif emotion == HAPPY2:
        count_emotion_state, emotion_state = play_sequence(happy2, 
                                                        count_emotion_state,
                                                        emotion_state)

        ###
        if emotion_state == len(happy2) - 1:
            emotion = BLINKING
        ###

    ###
    if time.time() - start_time > 15:
        emotion = HAPPY1
    ###

    pygame.display.flip()

pygame.quit()