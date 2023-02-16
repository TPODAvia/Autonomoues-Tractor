import pygame
import cv2
"""
this class will draw in pygame sdl 
images and telemetry

"""
class screen:
    def __init__(self,window,font_size):
        self.window = window
        self.font = pygame.font.SysFont(None, font_size)
        self.telemetry_text_position = (20,20)
    
    def display_image(self,img,postion):
        surf = pygame.surfarray.make_surface(img)
        surf = pygame.transform.rotate(surf, 270)
        text = self.font.render('hello\ntest', True, (255,0,0))
        self.window.blit(surf, postion)
        self.window.blit(text, self.telemetry_text_position)
        pygame.display.update()
