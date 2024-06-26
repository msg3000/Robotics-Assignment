# Handles logic of RRT and path search

import numpy as np
from mapping import WorldMapping
import random
import math
from PIL import Image, ImageOps
import matplotlib.pyplot as plt
import warnings


def binary_dilation_image(image, structuring_element):
  """
  Perform a dilation of a binary image with the given structuring element
  """
  image_h, image_w = image.shape
  struct_h, struct_w = structuring_element.shape

  pad_h = struct_h // 2
  pad_w = struct_w // 2
  # Pad the original image with zeros around the border
  padded_image = np.pad(image, ((pad_h, pad_h), (pad_w, pad_w)), mode='constant', constant_values=0)
  dilated_image = np.zeros_like(image)

  # Perform the dilation operation
  for i in range(image_h):
      for j in range(image_w):
          region = padded_image[i:i + struct_h, j:j + struct_w]
          if np.any(np.logical_and(region,structuring_element)):
              dilated_image[i, j] = 1
  return dilated_image


def bresenham_line(x0, y0, x1, y1):
    """
    Implements Bressenham line algorithm for constructing points along a line.
    Pseudocode from: https://www.baeldung.com/cs/bresenhams-line-algorithm
    """
    
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = err * 2
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    return points

def padding(image):
    # Convert the image to binary (1: obstacles, 0: free space)
    image = ImageOps.grayscale(image)
    binary_image = image.point(lambda p: p > 128 and 1)
    
    # Add padding to obstacles
    padding_size = 16 
    padded_binary_image = binary_dilation_image(1-np.array(binary_image), np.ones((padding_size,padding_size)))
    padded_binary_image = np.array(padded_binary_image)

    return padded_binary_image

class GridMapFromImage:
    """
    Handles logic of free space from image
    """
    def __init__(self, binary_image):
        self.binary_image = binary_image
        self.height, self.width = binary_image.shape
        
    def is_free_space(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.binary_image[y, x] == 0
        return False



class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0
        self.children = []

class RRT:
    """
    Implements RRT* algorithm
    """
    def __init__(self, start, goal, grid_map, world_map, step_size=1, neighbourhood_size = 2, verbose = True, animate = True):
        self.start = Node(*world_map.world_to_pixel(*start))
        self.goal = Node(*world_map.world_to_pixel(*goal))
        self.grid_map = grid_map
        self.world_map = world_map
        self.step_size = step_size
        self.neighbourhood_size = neighbourhood_size
        self.goal_sample_probs = 0.1 # Sample goal with some probability
        self.tree = [self.start]
        self.verbose = verbose
        self.animate = animate

        if self.verbose:
            warnings.filterwarnings("ignore",".*GUI is implemented.*")
            plt.plot(self.start.x, self.start.y, 'bo', markersize = 9, label = "Start")
            plt.text(self.start.x, self.start.y,'S', fontsize = 11)
            plt.plot(self.goal.x, self.goal.y, 'ro', markersize = 9, label = "Start")
            plt.text(self.goal.x , self.goal.y ,'G', fontsize =11)
            plt.pause(2)

    def get_random_point(self):
        """
        Generate a random point in the space
        """
        x,y = None, None
        if random.uniform(0,1) < self.goal_sample_probs:
            x = self.goal.x
            y = self.goal.y
        else:
            x = random.randint(105, 465)
            y = random.randint(60, 335)
        return x, y

    def get_nearest_node(self, point):
        """
        Compute nearest node in tree to a given point
        """
        nearest_node = None
        min_dist = float('inf')
        for node in self.tree:
            dist = self.euclidean_distance((node.x, node.y), point)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node
    
    def get_neighbourhood(self, point):
        """
        Extract all tree nodes within certain radius point
        """
        neighbourhood = []
        for node in self.tree:
            if self.euclidean_distance((node.x, node.y), point) <= self.neighbourhood_size and self.is_collision_free(node.x, node.y, point[0], point[1]):
                neighbourhood.append(node)
        return neighbourhood

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def is_collision_free(self, x, y, x1, y1):
        """
        Collision detection - generate all points along path using Bressenham algorithm, ensure none pass through an obstacle
        """
        points=bresenham_line(x,y,x1,y1)
        return np.all([self.grid_map.is_free_space(dx, dy) for dx,dy in points])
    
    def get_shortest_path_node(self, neighbourhood, point):
        return neighbourhood[np.argmin([node.cost + self.euclidean_distance((node.x, node.y), point) for node in neighbourhood])]

        
    def step(self):
        """
        Main RRT generation logic.
        - Sample new point
        - Test if valid
        - If valid, find nearest node and optimise:
            1. Set as parent node, node in tree which minimises its overall cost from start.
            2. Rewire tree to minimise overall cost from start of its neighbouring points
        """

        # Sample random point
        random_point = self.get_random_point()
        nearest_node = self.get_nearest_node(random_point)
        theta = math.atan2(random_point[1] - nearest_node.y, random_point[0] - nearest_node.x)
        new_x = int(nearest_node.x + self.step_size * math.cos(theta))
        new_y = int(nearest_node.y + self.step_size * math.sin(theta))
        
        if self.is_collision_free(nearest_node.x, nearest_node.y, new_x, new_y):
            new_node = Node(new_x, new_y)
            
            # Extract neighbourhood and optimise parent and cost
            neighbourhood = self.get_neighbourhood((new_x, new_y))
            parent_node = self.get_shortest_path_node(neighbourhood, (new_x, new_y))
            self.tree.append(new_node)
            new_node.parent = parent_node
            new_node.cost = parent_node.cost + self.euclidean_distance((parent_node.x, parent_node.y), (new_x, new_y))
            parent_node.children.append(new_node)
            

            # Rewire tree for optimality
            for node_neighbour in neighbourhood:
                tentative_cost = new_node.cost + self.euclidean_distance((node_neighbour.x, node_neighbour.y), (new_x,new_y))
                if node_neighbour.cost > tentative_cost :
                    node_neighbour.parent.children.remove(node_neighbour)
                    node_neighbour.parent = new_node
                    new_node.children.append(node_neighbour)
                    node_neighbour.cost = tentative_cost
            
            if self.verbose and self.animate:
                self.visualize(self.start)
                plt.plot(new_x, new_y, 'o', markersize = 6, color = "orange")
                plt.pause(0.0001)

            # Goal test
            if self.euclidean_distance((new_x, new_y), (self.goal.x, self.goal.y)) <= 2*self.step_size and self.is_collision_free(new_x, new_y, self.goal.x, self.goal.y):
                self.goal.parent = new_node
                new_node.children.append(self.goal)
                self.tree.append(self.goal)
                return True
            
        return False
    
    def visualize(self, root):
        plt.plot(root.x, root.y, 'go', markersize = 3)
        for child in root.children:
            plt.plot([root.x, child.x], [root.y, child.y], color='green', linestyle = '--')
            self.visualize(child)
        


    def build(self, max_steps=10000):
        """
        Build RRT 
        """
        for _ in range(max_steps):
            if self.step():
                return self.get_path()
        return None

    def get_path(self, display = False):
        """
        Reconfigure path from goal node
        """
        path = []
        node = self.goal
        while node.parent is not None:
            world_x, world_y = self.world_map.pixel_to_world(node.x, node.y)
            if self.verbose and display:
                plt.plot(node.parent.x, node.parent.y, 'go', markersize = 5)
                plt.plot([node.parent.x, node.x], [node.parent.y, node.y], color='red', linewidth =3)
                plt.pause(0.1)
            path.append((world_x, world_y))
            node = node.parent
        return path[::-1]  # Reverse the path

