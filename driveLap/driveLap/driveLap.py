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
			'/webserver_pkg/manual_drive',
			10)
		self.forward_distance_ = 0.0
		self.right_distance_ = 0.0
		self.left_distance_ = 0.0
		self.cur_dir = "none"

	def lidar_callback(self, msg):

		forward_distance_ = msg.ranges[0]
		#original true right should be 398
		right_distance_ = msg.ranges[408]
		#original left should be 132
		left_distance_ = msg.ranges[122]
		self.get_logger().info(f'Forward distance: {forward_distance_:.2f} meters')
		self.get_logger().info(f'Right distance: {right_distance_:.2f} meters')
		self.get_logger().info(f'Left distance: {left_distance_:.2f} meters')

		if forward_distance_ < 0.2:
			print("Stop")
			self.cur_dir = "stop"
			self.stop()
		elif (forward_distance_ < 2.0 and right_distance_ < 1.5) or (self.cur_dir == "right" and left_distance_ > 2.0 and forward_distance_ < 2.0):
			print("Left")
			self.cur_dir = "left"
			self.go_left()
		elif (forward_distance_ > 2.0 and (right_distance_ > 0.35 and right_distance_ < 1.0)):
			print("Straight")
			self.cur_dir = "straight"
			self.go_straight()
		elif (forward_distance_ < 2.0 and right_distance_ > 1.0):
			print("Right")
			self.cur_dir = "right"
			self.go_right()

	def go_straight(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.6
		self.cmd_vel_publisher.publish(input)

	def go_right(self):
		input = ServoCtrlMsg()
		input.angle = -1.0
		input.throttle = 0.8
		self.cmd_vel_publisher.publish(input)
	
	def go_left(self):
		input = ServoCtrlMsg()
		input.angle = 1.0
		input.throttle = 0.8
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
