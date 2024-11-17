import pygame
from air_hockey_game.src.air_hockey import AirHockey

import time
from pygame.locals import *

if __name__ == "__main__":
    air_hockey = AirHockey(render=True)
    while True:
        if any([event.type == 'QUIT' for event in pygame.event.get()]): break
        # time.sleep(0.05)
        air_hockey.step()
    pygame.quit()
