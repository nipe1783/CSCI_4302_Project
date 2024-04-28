import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg

class DriveLap(Node):
	def __init__(self):
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
		self.forward_distance_ = 0.0
		self.right_distance_ = 0.0
		self.left_distance_ = 0.0

	def lidar_callback(self, msg):

		forward_distance_ = msg.ranges[0]
		right_distance_ = msg.ranges[398]
		left_distance_ = msg.ranges[132]

		if(forward_distance_ > 1 and right_distance_ > .7):
			self.go_right()
		elif(forward_distance_ > 1 and right_distance_ > 0.5 and right_distance_ <= .7):
			self.go_straight()
		else:
			self.stop()

	def go_straight(self, msg):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.8
		self.cmd_vel_publisher.publish(input)

	def go_right(self, msg):
		input = ServoCtrlMsg()
		input.angle = -0.5
		input.throttle = 0.4
		self.cmd_vel_publisher.publish(input)

	def stop(self, msg):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.0
		self.cmd_vel_publisher.publish(input)
