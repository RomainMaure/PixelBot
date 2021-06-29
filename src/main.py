################################# IMPORTS #####################################

import pygame
from pygame.locals import *

################################## VARIABLES ##################################

# Eye blinking
blink = []
is_blinking = False
count_blinking = 0
count_blinking_state = 0
blinking_state = 0

BLINKING_TIME = 400
CHANGE_BLINK_STATE = 5


# General use
animation = True

FPS = 100
DIMENSIONS = (800, 480)

################################## MAIN #######################################

pygame.init()

window = pygame.display.set_mode(DIMENSIONS)
pygame.display.set_caption("PixelBot")

blink.append(pygame.image.load("../img/blink/blink_1.png").convert_alpha())
blink.append(pygame.image.load("../img/blink/blink_2.png").convert_alpha())
blink.append(pygame.image.load("../img/blink/blink_3.png").convert_alpha())
blink.append(pygame.image.load("../img/blink/blink_4.png").convert_alpha())
blink.append(pygame.image.load("../img/blink/blink_5.png").convert_alpha())

blink = [pygame.transform.scale(img, DIMENSIONS) for img in blink]
blink += reversed(blink)
blink.remove(blink[len(blink)//2])


clock = pygame.time.Clock()

while animation: 

    clock.tick(FPS)

    for event in pygame.event.get():
        if event.type == QUIT:
            animation = False

    # Blinking
    count_blinking += 1

    if count_blinking == BLINKING_TIME:
        is_blinking = True

    if is_blinking:
        count_blinking_state += 1

        if count_blinking_state == CHANGE_BLINK_STATE:
            blinking_state += 1
            count_blinking_state = 0

            if blinking_state == len(blink):
                count_blinking = 0
                blinking_state = 0
                is_blinking = False

    window.blit(blink[blinking_state], (0, 0))

    pygame.display.flip()

pygame.quit()