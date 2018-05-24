#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from gbot.msg import RobotCmd
import os
import sys
import time
import mplayer

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
        self.player = mplayer.Player('-nolirc -vo fbdev2:/dev/fb1 -vf scale=128:-3')
        #pygame.init()
        #rospy.loginfo("pygame init complete")
        #pygame.font.init()
        #pygame.mouse.set_cursor((8,8),(0,0),(0,0,0,0,0,0,0,0),(0,0,0,0,0,0,0,0))
        #pygame.mouse.set_visible(False)
        #self.disp = pygame.display.set_mode((HEIGHT, WIDTH))
        #rospy.loginfo("Display set mode complete")

        # Clear the display to a red background.
        # Can pass any tuple of red, green, blue values (from 0 to 255 each).
        self.y_offset = 10

        # Load default font.
        #self.font = pygame.font.Font(font, 16)
        #rospy.loginfo("Font load complete")

        rospy.Subscriber("robot_commands", RobotCmd, self.robot_state, queue_size=1)

    def quit(self):
        self.player.quit()

    def display_text(self, text):
        #textsurface = self.font.render(text, False, (200, 0, 0))
        
        #self.disp.blit(textsurface,(2, self.y_offset))
        #pygame.display.update()
        
        self.y_offset = self.y_offset + 28
        if self.y_offset > (HEIGHT - 28):
            self.y_offset = 10

    def display_image(self, filename):
        # Load an image.
        pass

    def play_movie(self, filename):
        self.player.loadfile(os.path.join(self.assets_dir, 'rising_sun.mpg'))

    def robot_state(self, data):
        self.display_text(data.cmd)

    def clear(self):
        pass
        #self.disp.fill((0,0,0))

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
