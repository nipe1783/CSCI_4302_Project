import rclpy
from rclpy.node import node
from sensor_msgs.mg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/rplidar_ros/scan',
            self.lidar_callback,
            10
        )

    #This is where we should process the Lidar readings
    def lidarcallback(self, msg):
        #for testing lets just log the size of the ranges array
        self.get_logger().info('Received a scan with {} measurements.'.format(len(msg.ranges)))

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