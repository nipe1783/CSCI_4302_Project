import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg

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

	def lidar_callback(self, msg):

		# front_right = msg.ranges[235:265:5]
		# front_left = msg.ranges[265:295:5]
		# min = float('inf')
  
		obstacle = False
		for i in range(200,330,5):
			dist = msg.ranges[i]
			if dist > .2 and dist < 1.3:
				obstacle = True

		forward_distance_ = msg.ranges[265]

		right_distance_ = msg.range[140]
		left_distance_ = msg.ranges[390]

		self.get_logger().info(f'Forward left: {forward_distance_:.2f} meters')
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
		elif (right_distance_ > 1.5):
			print("Right")
			self.cur_dir = "right"
			self.go_right(right_distance_)
		else:
			print("Hugging Wall")
			self.cur_dir = "hug_wall"
			self.go_straight(right_distance_-1)

	def go_straight(self,error):
		input = ServoCtrlMsg()
		# factor = (error) * .5
		input.angle = error
		input.throttle = -0.7
		self.cmd_vel_publisher.publish(input)

	def reverse(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.5
		self.cmd_vel_publisher.publish(input)

	def go_right(self,right_distance):
		input = ServoCtrlMsg()
		input.angle = 0.7
		input.throttle = -0.6
		self.cmd_vel_publisher.publish(input)
	
	def go_left(self,left_distance):
		input = ServoCtrlMsg()
		input.angle = -0.7
		input.throttle = -0.6
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
