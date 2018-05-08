#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont
import ST7735 as TFT
import Adafruit_GPIO as GPIO
import Adafruit_GPIO.SPI as SPI


WIDTH = 128
HEIGHT = 160
SPEED_HZ = 4000000

# Raspberry Pi configuration.
DC = 24
RST = 25
SPI_PORT = 0
SPI_DEVICE = 0

class Display:
    def __init__(self):
        self.disp = TFT.ST7735(
            DC,
            rst=RST,
            spi=SPI.SpiDev(
                SPI_PORT,
                SPI_DEVICE,
                max_speed_hz=SPEED_HZ))

        # Initialize display.
        self.disp.begin()

        # Clear the display to a red background.
        # Can pass any tuple of red, green, blue values (from 0 to 255 each).
        self.disp.clear()

        # Load default font.
        self.font = ImageFont.load_default()

        # Alternatively load a TTF font.
        # Some other nice fonts to try: http://www.dafont.com/bitmap.php
        #font = ImageFont.truetype('Minecraftia.ttf', 16)

        rospy.Subscriber("robot_commands", String, self.robot_state, queue_size=1)

    # Define a function to create rotated text.  Unfortunately PIL doesn't have good
    # native support for rotated fonts, but this function can be used to make a
    # text image and rotate it so it's easy to paste in the buffer.
    def draw_rotated_text(self, image, text, position, angle, font, fill=(255,255,255)):
        # Get rendered font width and height.
        draw = ImageDraw.Draw(image)
        width, height = draw.textsize(text, font=font)
        # Create a new image with transparent background to store the text.
        textimage = Image.new('RGBA', (width, height), (0,0,0,0))
        # Render the text.
        textdraw = ImageDraw.Draw(textimage)
        textdraw.text((0,0), text, font=font, fill=fill)
        # Rotate the text image.
        rotated = textimage.rotate(angle, expand=1)
        # Paste the text into the image, using it as a mask for transparency.
        image.paste(rotated, position, rotated)

    def robot_state(self, text):
        self.draw_rotated_text(self.disp.buffer, text, (150, 120), 90, self.font, fill=(255,255,255))
        self.disp.display()

if __name__ == "__main__":
    rospy.init_node('display')
 
    display = Display()
    rospy.spin()
