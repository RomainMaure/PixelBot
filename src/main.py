################################# IMPORTS #####################################

import pygame
from pygame.locals import *

############################### GLOBAL VARIABLES ##############################

continuer = True
FPS = 100
DIMENSIONS = (800, 480)
WHITE = (255, 255, 255)

################################## MAIN #######################################

pygame.init()

fenetre = pygame.display.set_mode(DIMENSIONS)
pygame.display.set_caption("PixelBot")

fond = pygame.Surface(DIMENSIONS)
fond.fill(WHITE)

fenetre.blit(fond, (0, 0))

clock = pygame.time.Clock()

while continuer: 

    clock.tick(FPS)

    for event in pygame.event.get():
        if event.type == QUIT:
            continuer = False

    pygame.display.flip()

pygame.quit()