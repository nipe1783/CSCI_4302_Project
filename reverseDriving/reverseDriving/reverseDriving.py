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
  
		obstacle = False
		# for i in range(200,330,5):
		# 	dist = msg.ranges[i]
		# 	if dist < 1:
		# 		obstacle = False

		forward_distance_ = msg.ranges[265]

		right_distance_ = msg.ranges[140]
		left_distance_ = msg.ranges[390]

		self.get_logger().info(f'Forward: {forward_distance_:.2f} meters')
		self.get_logger().info(f'Right distance: {right_distance_:.2f} meters')
		self.get_logger().info(f'Left distance: {left_distance_:.2f} meters')

		if forward_distance_ < 0.2:
			print("Stop")
			self.cur_dir = "stop"
			self.stop()
		elif (obstacle):
			print("Obstacle")
			self.cur_dir = "avoiding"
			self.go_left()
		elif (right_distance_ > 2):
			print("Right")
			self.cur_dir = "right"
			self.go_right()
		else:
			print("Hugging Wall")
			self.cur_dir = "hug_wall"
			self.go_straight(math.sqrt(right_distance_-1))

	def go_straight(self,error):
		input = ServoCtrlMsg()
		# factor = (error) * .5
		input.angle = error
		input.throttle = self.max_throttle
		self.cmd_vel_publisher.publish(input)

	def reverse(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.5
		self.cmd_vel_publisher.publish(input)

	def go_right(self):
		input = ServoCtrlMsg()
		input.angle = 0.7
		input.throttle = -self.max_throttle * .6
		self.cmd_vel_publisher.publish(input)
	
	def go_left(self):
		input = ServoCtrlMsg()
		input.angle = -0.7
		input.throttle = self.max_throttle * .75
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
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
