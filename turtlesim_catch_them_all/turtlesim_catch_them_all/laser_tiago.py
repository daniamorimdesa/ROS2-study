import rclpy
from rclpy.node import Node

import time
import math

from sensor_msgs.msg import LaserScan


class LaserClass(Node):

	def __init__(self):
		super().__init__('laser_node')
		
		# Subscriber
		self.create_subscription(LaserScan, '/scan_raw', self.laser_cb, 1)
		
	def laser_cb(self, msg):
		# read ranges and pick the minimum value and its index
		min_range = min(msg.ranges)
		min_index = msg.ranges.index(min_range)
		
		# read angle increment and calculate the angle corresponding to the minimum range
		angle_increment = msg.angle_increment
		angle_min = msg.angle_min
		angle_at_min_range = angle_min + min_index * angle_increment
		
		# convert the polar coordinates (min_range, angle_at_min_range) to Cartesian coordinates (x, y)
		x = min_range * math.cos(angle_at_min_range)
		y = min_range * math.sin(angle_at_min_range)
		
		# print coordinates in cartesian
		print(f"Closest obstacle at x: {x:.2f} m, y: {y:.2f} m")
		# print distance
		print(f"Distance to closest obstacle: {min_range:.2f} m")


def main(args=None):
    rclpy.init(args=args)

    laser = LaserClass()
    rclpy.spin(laser)
	
    laser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
		