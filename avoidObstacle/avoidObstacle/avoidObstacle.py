import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from deepracer_interfaces_pkg.msg import ServoCtrlMsg
import time

# Get the current time in seconds since the epoch (January 1, 1970)
current_time = time.time()

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


		self.psi = 0.0
		self.th = 0.0
		self.df_cut = 0.2
		self.th_max = 0.8
		self.ds_min = 0.2
		self.psi_max = 1.0
		self.psi_min = -1.0
		self.t_start = time.time()
		self.t_prev = time.time()
		self.ds_prev = 0.0
		self.psi_acc = 0.0



	def lidar_callback(self, msg):

		df = msg.ranges[0]
		#original true right should be 398
		dr = msg.ranges[398]
		#original left should be 132
		dl = msg.ranges[132]

		if df == float('inf'):
			df = 0
		if dr == float('inf'):
			dr = 0
		if dl == float('inf'):
			dl = 0

		#ds = dl-dr
		ds = 1-dr
		self.get_logger().info(f'Forward distance: {df:.2f} meters')
		self.get_logger().info(f'Right distance: {dr:.2f} meters')
		self.get_logger().info(f'Left distance: {dl:.2f} meters')

		#PID gains
		k_th_p = 1.0
		k_th_d = 0.0
		k_th_i = 0.0

		k_psi_p = 0.5
		k_psi_d = 0.1
		k_psi_i = 0.0

		#P
		if df < self.df_cut:
			th_p = 0.0
		else:
			th_p = 0.6 #df

		'''if dl < self.ds_min or dr < self.ds_min:
			th_p = 0.0
			psi_p = 0.0
		else:
			psi_p = ds
		'''

		#D
		t = time.time()
		dt = t-self.t_prev
		th_d = 0.0
		psi_d = (ds-self.ds_prev)/dt

		#I
		th_i = 0.0
		psi_i = self.psi_acc+ds*dt

		

		# update prev vals
		self.ds_prev = ds
		self.t_prev = t
		self.psi_acc = psi_i
		
		#apply gain and sum
		self.th = th_p*k_th_p + th_d*k_th_p + th_i*k_th_i
		self.psi = psi_p*k_psi_p + psi_d*k_psi_p + psi_i*k_psi_i

		message0 = f"Raw Actions: Throttle= {self.th} Steering Angle= {self.psi}"
		
		#cuttoff
		if self.th > self.th_max:
			self.th = self.th_max

		if self.psi > self.psi_max:
			self.psi = self.psi_max
		elif self.psi < self.psi_min:
			self.psi = self.psi_min

		self.act()
		
		message1 = f"Actual Actions: Throttle= {self.th} Steering Angle= {self.psi}"
		message2 = f"LIDAR: left= {dl} right= {dr} front= {df}"
		
		# Print the message
		print(message0)
		print(message1)
		print(message2)

	def act(self):
		input = ServoCtrlMsg()
		input.angle = float(self.psi)
		input.throttle = float(self.th)
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
