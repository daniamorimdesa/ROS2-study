import rclpy
from rclpy.node import Node

import time
import math

from geometry_msgs.msg import Twist
# from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class TiagoClass(Node):

	def __init__(self):
		super().__init__('tiago_node')
		
		# Subscriber
		self.create_subscription(Odometry, '/ground_truth_odom', self.pose_cb, 1)
		
		# Publisher
		self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 1)
		self.twist_cmd = Twist()
		
		# To detect the first loop
		self.firstLoop = True
		
		# To identify the maneuver
		self.rotationIsOver = False
		
	def pose_cb(self, msg):
		
		# Extract relevant information
		x = msg.pose.pose.position.x
		y = msg.pose.pose.position.y

		qtn = msg.pose.pose.orientation
		qtn_list = [qtn.x, qtn.y, qtn.z, qtn.w]
		(roll, pitch, yaw) = euler_from_quaternion(qtn_list)
		print(x, y, yaw)
		
		
		# # Goal
		# x_goal = 2
		# y_goal = 3
		
		# # Save intial pose
		# if self.firstLoop:
		# 	# Initial position
		# 	self.x_init = msg.x
		# 	self.y_init = msg.y
			
		# 	# Compute desired orientation
		# 	self.theta_goal = math.atan2(y_goal - self.y_init, x_goal - self.x_init)
		# 	#print(math.degrees(theta_goal))
			
		# 	# Compute desired distance
		# 	self.distance_goal = math.sqrt((x_goal - self.x_init)**2 + (y_goal - self.y_init)**2)
			
		# 	#print(distance_goal)
			
		# 	self.firstLoop = False
			
		# # Update the current position
		# self.x = msg.x
		# self.y = msg.y
		# self.theta = msg.theta
		
		# print(self.x,self.y)
		
		# # Controller
		# if not self.rotationIsOver:
		# 	print("Step 1")
		# 	K = 1
		# 	self.twist_cmd.angular.z = K*(self.theta_goal - self.theta)
		# 	#print(self.theta_goal - self.theta)
			
		# 	# Detect the end of the rotation
		# 	if abs(self.theta_goal - self.theta) < 0.001:
		# 		self.rotationIsOver = True

		# else:
		# 	print("Step 2")
		# 	# Compute the current distance
		# 	distance = math.sqrt((x_goal - self.x)**2 + (y_goal - self.y)**2)

		# 	K = 0.1
		# 	self.twist_cmd.angular.z = 0.0
		# 	self.twist_cmd.linear.x = K*(distance)
		
		# # Publish the velocity
		# self.twist_pub.publish(self.twist_cmd)
		
		
	def test_twist(self):
		# Start
		self.twist_cmd.linear.x = 0.2	
		self.twist_pub.publish(self.twist_cmd)
		
		time.sleep(1)
		
		# Stop
		self.twist_cmd.linear.x = 0.0	
		self.twist_pub.publish(self.twist_cmd)
		
def main(args=None):
    rclpy.init(args=args)

    my_tiago = TiagoClass()

    rclpy.spin(my_tiago)
    #my_turtle.test_twist()


    my_tiago.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
		