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

	def lidar_callback(self, msg):

		forward_distance_ = msg.ranges[0]
		right_distance_ = msg.ranges[398]
		top_right_distance_ = msg.ranges[486]
		top_left_distance = msg.ranges[44]
		left_distance_ = msg.ranges[132]
		self.get_logger().info(f'Forward distance: {forward_distance_:.2f} meters')
		self.get_logger().info(f'Right distance: {right_distance_:.2f} meters')
		self.get_logger().info(f'Left distance: {left_distance_:.2f} meters')

		if(forward_distance_ > 2 and right_distance_ > 1):
			print("Right")
			self.go_right()
		elif(forward_distance_ > 2 and right_distance_ > 1 and right_distance_ <= 1):
			print("Straight")
			self.go_straight()
		elif(forward_distance_ < 2 and right_distance_ <= 1 and left_distance_ > 2):
			print("Left")
			self.go_left()
		else:
			print("Stop")
			self.stop()

	def go_straight(self):
		input = ServoCtrlMsg()
		input.angle = 0.0
		input.throttle = 0.7
		self.cmd_vel_publisher.publish(input)

	def go_right(self):
		input = ServoCtrlMsg()
		input.angle = -0.7
		input.throttle = 0.7
		self.cmd_vel_publisher.publish(input)
	
	def go_left(self):
		input = ServoCtrlMsg()
		input.angle = 0.7
		input.throttle = 0.7
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
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
