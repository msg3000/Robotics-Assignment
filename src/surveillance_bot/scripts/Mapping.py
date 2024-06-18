import numpy as np
from PIL import Image


class WorldMapping:

    def __init__(self, resolution, origin):
        self.origin= origin
        self.image=Image.open('/home/shnifel/ros_home/robot_assignment_ws/map_2.jpg')
        self.height= self.image.size[1]
        self.width= self.image.size[0]
        self.resolution = resolution

    def pixel_to_world(self,px, py):
        wx = self.origin[0] + px * self.resolution
        wy = self.origin[1] + (self.height - py) * self.resolution
        return wx, wy

    def world_to_pixel(self,wx, wy):
        px =  (wx - self.origin[0]) / self.resolution
        py =  ( self.origin[1] - wy + self.height * self.resolution) / self.resolution
        return px, py


