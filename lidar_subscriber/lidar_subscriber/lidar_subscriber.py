import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg

class LidarSubscriber(Node):
	def __init__(self):
		super().__init__('lidar_subscriber')
		self.subscription = self.create_subscription(
			LaserScan,
			'/rplidar_ros/scan',
			self.lidar_callback,
			10)
		self.cmd_vel_publisher = self.create_publisher(
			ServoCtrlMsg,
			'/webserver_pkg/manual_drive',
			10)

	#This is where we should process the Lidar readings
	def lidar_callback(self, msg):
		print(len(msg.ranges))
		forward_distance = self.get_forward_distance(msg)
		self.get_logger().info(f'Forward distance: {forward_distance:.2f} meters')
		if forward_distance > 1.0:
			self.drive_forward()
		else:
			self.stop()

	def get_forward_distance(self, scan: LaserScan) -> float:
		forward_distance = scan.ranges[0]
		# Filtering out invalid measurements
		if forward_distance == float('inf') or forward_distance == 0.0:
			return -1.0
		return forward_distance

	def drive_forward(self) -> float:
		msg = ServoCtrlMsg()
		msg.angle = 0.0 #no steer
		msg.throttle = 0.6
		self.cmd_vel_publisher.publish(msg)

	def stop(self):
		msg = ServoCtrlMsg()
		msg.angle = 0.0
		msg.throttle = 0.0
		self.cmd_vel_publisher.publish(msg)

def main(args = None):
    rclpy.init(args=args)
    node = LidarSubscriber()
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
