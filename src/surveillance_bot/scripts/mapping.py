import numpy as np
from PIL import Image
import os

class WorldMapping:
    """
    Handles logic of pixel to world mapping
    """

    def __init__(self, resolution, origin):
        # Configure image and resolution for mapping
        self.origin= origin
    
        self.image=Image.open(os.getcwd() + '/maps/map_2_final.jpg')
        self.height= self.image.size[1]
        self.width= self.image.size[0]
        self.resolutiony = 28.8/self.height
        self.resolutionx = 28.8/self.width 

    def pixel_to_world(self,px, py):
        wx = self.origin[0] + px * self.resolutionx
        wy = self.origin[1] + (self.height-py) * self.resolutiony
        return wx, wy

    def world_to_pixel(self,wx, wy):
        px =  int((wx - self.origin[0]) / self.resolutionx)
        py =  int(( self.origin[1] - wy + self.height * self.resolutiony) / self.resolutiony)
        return px, py


