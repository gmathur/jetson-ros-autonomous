#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pygame
import os
import sys
import time
from pygame.locals import *

#os.environ['SDL_VIDEODRIVER'] = 'dummy' 
os.environ["SDL_FBDEV"] = "/dev/fb1"
os.environ['SDL_NOMOUSE'] = "1"

WIDTH = 128
HEIGHT = 128
SPEED_HZ = 4000000
FPS = 24

class Display:
    def __init__(self, font, assets_dir):
        rospy.loginfo("Loading font %s assets dir %s", font, assets_dir)

        self.assets_dir = assets_dir

        pygame.init()
        rospy.loginfo("pygame init complete")
        pygame.font.init()
        pygame.mouse.set_cursor((8,8),(0,0),(0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0))
        pygame.mouse.set_visible(False)
        self.disp = pygame.display.set_mode((HEIGHT, WIDTH))
        rospy.loginfo("Display set mode complete")

        # Clear the display to a red background.
        # Can pass any tuple of red, green, blue values (from 0 to 255 each).
        self.y_offset = 10

        # Load default font.
        self.font = pygame.font.Font(font, 16)
        rospy.loginfo("Font load complete")

        rospy.Subscriber("robot_commands", String, self.robot_state, queue_size=1)

    def quit(self):
        pygame.quit()

    def display_text(self, text):
        textsurface = self.font.render(text, False, (200, 0, 0))
        
        self.disp.blit(textsurface,(2, self.y_offset))
        pygame.display.update()
        
        self.y_offset = self.y_offset + 28
        if self.y_offset > (HEIGHT - 28):
            self.y_offset = 10

    def display_image(self, filename):
        # Load an image.
        image = pygame.image.load("%s/%s" % (self.assets_dir, filename))

        self.disp.blit(image, [0,0])

        pygame.display.update()

    def play_movie(self, filename):
        pygame.mixer.quit()
        rospy.loginfo("Mixer quit")
        movie = pygame.movie.Movie("%s/%s" % (self.assets_dir, filename))
        rospy.loginfo("Movie loaded")
        background = pygame.Surface((HEIGHT, WIDTH))
        rospy.loginfo("Surface created")

        self.disp.blit(background, (0, 0))
        pygame.display.update()

        mrect = pygame.Rect(0,0,128,100)
        movie.set_display(self.disp, mrect.move(1, 1))
        movie.play()

        rospy.loginfo("Playing movie now")
        while movie.get_busy() and movie.get_time() <= 6:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    break

        rospy.loginfo("Finished playing movie")
        movie.stop()
        pygame.display.update()

    def robot_state(self, data):
        self.display_text(data.data)

    def clear(self):
        self.disp.fill((0,0,0))

if __name__ == "__main__":
    rospy.init_node('display')
    font = rospy.get_param("font")
    assets_dir = rospy.get_param("assets")
    display = Display(font, assets_dir)
    
    try:
        display.play_movie("rising_sun.mpg")
        time.sleep(7)
        display.clear()
        rospy.spin()
    finally:
        display.quit()
