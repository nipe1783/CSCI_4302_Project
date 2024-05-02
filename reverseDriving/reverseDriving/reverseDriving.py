import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import math

class ReverseDrive(Node):
	def __init__(self):
		super().__init__('reverseDrive')
		self.subscription = self.create_subscription(
			LaserScan,
			'/rplidar_ros/scan',
			self.lidar_callback,
			10)
		self.cmd_vel_publisher = self.create_publisher(
			ServoCtrlMsg,
			'/ctrl_pkg/servo_msg',
			10)
		self.forward_distance_ = 0.0
		self.right_distance_ = 0.0
		self.left_distance_ = 0.0
		self.cur_dir = "none"
		self.max_throttle = -.6

	def lidar_callback(self, msg):

		# front_right = msg.ranges[235:265:5]
		# front_left = msg.ranges[265:295:5]
		# min = float('inf')

		forward_distance_ = msg.ranges[265]

		right_distance_ = min(msg.ranges[110:155:5])
		left_distance_ = min(msg.ranges[380:420:5])

		obstacle = False
		dist = min(msg.ranges[230:305:5])
		if dist < 1 and right_distance_>.35:
			obstacle = True

		self.get_logger().info(f'Forward: {dist:.2f} meters')
		self.get_logger().info(f'Right distance: {right_distance_:.2f} meters')
		self.get_logger().info(f'Left distance: {left_distance_:.2f} meters')
		self.get_logger().info(f'Min Forward Cone: {dist:.2f} meters')
		# self.get_logger().info(f'Velocity: {dist:.2f} meters')

		if forward_distance_ < 0.3:
			print("Stop")
			self.cur_dir = "stop"
			self.stop()
		elif (obstacle):
			print("Obstacle")
			self.cur_dir = "avoiding"
			self.go_right()
		elif (right_distance_ > 2):
			print("Right")
			self.cur_dir = "right"
			self.go_right()
		else:
			print("Hugging Wall")
			self.cur_dir = "hug_wall"
			self.hug_wall((1-right_distance_))

	def hug_wall(self,error):
		input = ServoCtrlMsg()
		input.angle = error**3
		input.throttle = self.max_throttle
		print(input.throttle)
		self.cmd_vel_publisher.publish(input)

	def reverse(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.5
		print(input.throttle)
		self.cmd_vel_publisher.publish(input)

	def go_right(self):
		input = ServoCtrlMsg()
		input.angle = 0.5
		input.throttle = self.max_throttle * .5
		print(input.throttle)
		self.cmd_vel_publisher.publish(input)
	
	def go_left(self):
		input = ServoCtrlMsg()
		input.angle = -0.6
		input.throttle = self.max_throttle * .5
		print(input.throttle)
		self.cmd_vel_publisher.publish(input)

	def go_straight(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = self.max_throttle
		self.cmd_vel_publisher.publish(input)

	def stop(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.0
		self.cmd_vel_publisher.publish(input)

def main(args = None):
    rclpy.init(args=args)
    node = ReverseDrive()
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
