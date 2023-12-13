import numpy as np
import rospy
import yaml
from nav_msgs.msg import OccupancyGrid


class FactorySimulator:
    def __init__(self):
        rospy.init_node('factory_sim', anonymous = True)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size = 10)
        self.dimensions = self.load_dimensions()
    def load_dimensions(self):
        with open('factory_dim.yaml', 'r') as file:  # dimensions here, in read mode
            dimensions = yaml.safe_load(file)
        return dimensions
    def generate_environment(self):
        length = int(self.dimensions['length'] * 5)
        width = int(self.dimensions['width'] * 5)

        # Create an empty map
        occupancy_grid = np.zeros((length, width), dtype = np.int32)

        # Add obstacles
        for obstacle in self.dimensions['obst']:
            y = int(obstacle['y'] * 15)
            x = int(obstacle['x'] * 15)
            radius = int(obstacle['radius'] * 15)
            self.add_circle(occupancy_grid, x, y, radius)

        # Create map
        map1 = OccupancyGrid()
        map1.header.frame_id = 'factory1'
        map1.info.resolution = 0.2
        map1.info.height = length
        map1.info.width = width
        map1.data = list(occupancy_grid.flatten())
        self.map_pub.publish(map1)

    def add_circle(self, grid, x, y, radius):  # add circular obstacles
        y, x = np.ogrid[-x:grid.shape[0] - x, -y:grid.shape[1] - y]
        mask = x * x + y * y <= radius * radius
        grid[mask] = 50

    if __name__ == '__main__':
        try:
            simulator = FactorySimulator()
            simulator.generate_environment()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
