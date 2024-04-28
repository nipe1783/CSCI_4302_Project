import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
from collections import deque

class DriveLap(Node):
	def__init__(self):
		super().__init__('drive lap')
		self.subscription = self.create_subscription(
			LaserScan,
			'/rplidar_ros/scan',
			self.lidar_callback,
			10)
		self.cmd_vel_publisher = self.create_publisher(
			ServoCtrlMsg,
			'/webserver_pkg/manual_drive',
			10)
		self.driving_forward = True
		self.lidar_buffer = deque(maxlen=50) #Buffer to store the last 50 LIDAR readings

	def lidar_callback(self, msg):
		#Update lidar buffer with current readings
		self.lidar_buffer.append(msg.ranges)

		#Analyze lidar buffer to detect corners and intersections
		recent_readings = list(self.lidar_buffer)[-10:]
		if (not self.detect_intersection(recent_readings) and not self.detect_corner(recent_readings)):
			self.drive_forward(msg)
		elif self.detect_intersection(recent_readings):
			self.go_straight(msg)
		elif self.detect_corner(recent_readings):
			self.turn(msg)

	def get_forward_distance(self, scan: LaserScan):
		forward_distance = scan.ranges[0]
		if forward_distance == float('inf') or forward_distance == 0.0:
			return -1.0
		return forward_distance 

	def detect_intersection(self, readings):
		#Detect if it is an intersection based on the pattern of readings
		#Implement logic
		pass

	def detect_corner(self, readings):
		#Detect if it is a corner based on the pattern of readings
		#Implement logic
		pass

	def go_straight(self, msg):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.8
		self.cmd_vel_publisher.publish(input)

	def turn(self, msg):
		#Implemetn turn algorithm
		pass

	def drive_forward(self, msg):
		forward_distance = self.get_forward_distance(msg)
		self.get_logger().info(f'Forward distance: {forward_distance:.2f} meters')
		input = ServoCtrlMsg()
		input.angle = 0.0
		if forward_distance > 1.0:
			input.throttle = 0.8
		else:
			input.throttle = 0.0
		self.cmd_vel_publisher.publish(input)
