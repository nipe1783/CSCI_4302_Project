import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import time

class DriveLap(Node):
	def __init__(self):
		super().__init__('driveLap')
		self.subscription = self.create_subscription(
			LaserScan,
			'/rplidar_ros/scan',
			self.lidar_callback,
			10)
		self.cmd_vel_publisher = self.create_publisher(
			ServoCtrlMsg,
			'/ctrl_pkg/servo_msg',
			10)
		self.forward_distance = 0.0
		self.right_distance = 0.0
		self.left_distance = 0.0
		self.max_throttle = 0.55
		self.cur_dir = "none"

	def lidar_callback(self, msg):
		forward_distance = msg.ranges[0]
		right_45 = msg.ranges[464]
		right_distance = msg.ranges[398]
		left_45 = msg.ranges[66]
		left_distance = msg.ranges[132]
		self.get_logger().info(f'Forward distance: {forward_distance:.2f} meters')
		self.get_logger().info(f'Right distance: {right_distance:.2f} meters')
		self.get_logger().info(f'Right 45 distance: {right_45:.2f} meters')
		self.get_logger().info(f'Left distance: {left_distance:.2f} meters')
		self.get_logger().info(f'Left 45 distance: {left_45:.2f} meters')

		if forward_distance < 0.2:
			print("Stop")
			self.cur_dir = "stop"
			self.stop()
		# or (self.cur_dir == "right" and left_distance_ > 2.0 and forward_distance_ < 2.0)
		elif (forward_distance < 2.0 and right_distance < 2.0) or (self.cur_dir == "right" and left_distance > 2.0 and forward_distance < 1.0) or (right_45 < 0.9):
			print("Left")
			self.cur_dir = "left"
			self.go_left()
			print("sleeping")
			time.sleep(0.1)
		elif (right_distance < 0.4 and forward_distance > 2.0):
			print("Stabilize")
			self.cur_dir = "stabilize"
			self.stabilize()
		elif (right_distance > 0.35 and right_distance < 1.0):
			print("Straight")
			self.cur_dir = "straight"
			self.go_straight()
		elif (right_distance > 1.5) or (left_45 < 0.9):
			print("Right")
			self.cur_dir = "right"
			self.go_right()
			print("sleeping")
			time.sleep(0.1)
		else:
			print("Default Straight (turn slightly right)")
			self.cur_dir = "stabilize"
			self.stabilizeRight()

	def go_straight(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = self.max_throttle
		self.cmd_vel_publisher.publish(input)

	def stabilizeRight(self):
		input = ServoCtrlMsg()
		input.angle = -0.05
		input.throttle = self.max_throttle
		self.cmd_vel_publisher.publish(input)

	def go_right(self):
		input = ServoCtrlMsg()
		input.angle = -0.6
		input.throttle = self.max_throttle
		self.cmd_vel_publisher.publish(input)
	
	def go_left(self):
		input = ServoCtrlMsg()
		input.angle = 0.6
		input.throttle = self.max_throttle
		self.cmd_vel_publisher.publish(input)

	def stabilize(self):
		input = ServoCtrlMsg()
		input.angle = 0.05
		input.throttle = self.max_throttle
		self.cmd_vel_publisher.publish(input)

	def stop(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.0
		self.cmd_vel_publisher.publish(input)

def main(args = None):
    rclpy.init(args=args)
    node = DriveLap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
