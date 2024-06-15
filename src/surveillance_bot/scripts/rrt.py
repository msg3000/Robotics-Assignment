
import numpy as np
from Mapping import WorldMapping
import random
import math
from PIL import Image, ImageOps

WorldMap=WorldMapping(0.05,[-13,-3,0])
image = WorldMap.image


def padding(image):
    # Convert the image to binary (0: obstacles, 255: free space)
    binary_image = image.point(lambda p: p > 128 and 255)

    # Add padding to obstacles
    padding_size = 5  # Adjust this value as needed
    padded_binary_image = ImageOps.expand(binary_image, border=padding_size, fill=0)

    # Convert padded image to numpy array
    padded_binary_image = np.array(padded_binary_image)

    return padded_binary_image

class GridMapFromImage:
    def __init__(self, binary_image):
        self.binary_image = binary_image
        self.height, self.width = binary_image.shape

    def is_free_space(self, x, y):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.binary_image[y, x] == 255
        return False

padded_binary_image = padding(image)
# Initialize the grid map from the padded binary image
grid_map_obj = GridMapFromImage(padded_binary_image)



class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

class RRT:
    def __init__(self, start, goal, grid_map, world_map, step_size=1):
        self.start = Node(*world_map.world_to_pixel(*start))
        self.goal = Node(*world_map.world_to_pixel(*goal))
        self.grid_map = grid_map
        self.world_map = world_map
        self.step_size = step_size
        self.tree = [self.start]

    def get_random_point(self):
        x = random.randint(0, self.grid_map.width - 1)
        y = random.randint(0, self.grid_map.height - 1)
        return x, y

    def get_nearest_node(self, point):
        nearest_node = None
        min_dist = float('inf')
        for node in self.tree:
            dist = self.euclidean_distance((node.x, node.y), point)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def is_collision_free(self, x, y):
        return self.grid_map.is_free_space(x, y)

    def step(self):
        random_point = self.get_random_point()
        nearest_node = self.get_nearest_node(random_point)
        theta = math.atan2(random_point[1] - nearest_node.y, random_point[0] - nearest_node.x)
        new_x = int(nearest_node.x + self.step_size * math.cos(theta))
        new_y = int(nearest_node.y + self.step_size * math.sin(theta))
        
        if self.is_collision_free(new_x, new_y):
            new_node = Node(new_x, new_y)
            new_node.parent = nearest_node
            self.tree.append(new_node)
            if self.euclidean_distance((new_x, new_y), (self.goal.x, self.goal.y)) <= self.step_size:
                self.goal.parent = new_node
                self.tree.append(self.goal)
                return True
        return False

    def build(self, max_steps=1000):
        for _ in range(max_steps):
            if self.step():
                return self.get_path()
        return None

    def get_path(self):
        path = []
        node = self.goal
        while node is not None:
            world_x, world_y = self.world_map.pixel_to_world(node.x, node.y)
            path.append((world_x, world_y))
            node = node.parent
        return path[::-1]  # Reverse the path

# Define start and goal coordinates in world coordinates
start_world = (0, 0)  # Example start coordinate in world coordinates
goal_world = (4, 4)  # Example goal coordinate in world coordinates

# Initialize RRT with world coordinates
rrt = RRT(start_world, goal_world, grid_map_obj, WorldMap, step_size=10)

# Build RRT and get the path
path = rrt.build()
print("Path in world coordinates:", path)
