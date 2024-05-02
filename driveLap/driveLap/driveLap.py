import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg

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
		self.cur_dir = "none"

	def lidar_callback(self, msg):
		forward_distance = msg.ranges[265]
		right_45 = msg.ranges[199]
		right_distance = msg.ranges[132]
		left_45 = msg.ranges[332]
		left_distance = msg.ranges[398]
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
		elif (forward_distance < 2.0 and right_distance < 1.5) or (self.cur_dir == "right" and left_distance > 2.0 and forward_distance < 1.0) or (right_45 < 0.8):
			print("Left")
			self.cur_dir = "left"
			for x in range(10):
				self.go_left()
		elif (right_distance < 0.4 and forward_distance > 2.0):
			print("Stabilize")
			self.cur_dir = "stabilize"
			self.stabilize()
		elif (right_distance > 0.35 and right_distance < 1.0):
			print("Straight")
			self.cur_dir = "straight"
			self.go_straight()
		elif (right_distance > 1.5) or (left_45 < 0.8):
			print("Right")
			self.cur_dir = "right"
			for x in range(10):
				self.go_right()
		else:
			print("Default Straight")
			self.cur_dir = "straight"
			self.go_straight()

	def go_straight(self):
		input = ServoCtrlMsg()
		input.angle = 0.1
		input.throttle = -0.6
		self.cmd_vel_publisher.publish(input)

	def go_right(self):
		input = ServoCtrlMsg()
		input.angle = 0.5
		input.throttle = -0.6
		self.cmd_vel_publisher.publish(input)
	
	def go_left(self):
		input = ServoCtrlMsg()
		input.angle = -0.35
		input.throttle = -0.6
		self.cmd_vel_publisher.publish(input)

	def stabilize(self):
		input = ServoCtrlMsg()
		input.angle = -0.05
		input.throttle = -0.6
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
