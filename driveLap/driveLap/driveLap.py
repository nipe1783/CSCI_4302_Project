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
		self.max_throttle = 0.68

		self.avoid_toggle = 0




		# for PID implementation
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
		self.dr_lp = 0
		self.wall_dist = 0.7
		

	def lidar_callback(self, msg):
		forward_distance = msg.ranges[0]
		right_45 = msg.ranges[464]
		right_15 = msg.ranges[420]
		right_distance = msg.ranges[398]
		left_45 = msg.ranges[66]
		left_distance = msg.ranges[132]
		#for x in range(msg.ranges):
			#if msg.ranges[x] == float('inf'):
				#msg.ranges[x] = 80.0
		self.get_logger().info(f'Forward distance: {forward_distance:.2f} meters')
		self.get_logger().info(f'Right distance: {right_distance:.2f} meters')
		#self.get_logger().info(f'Right 45 distance: {right_45:.2f} meters')
		self.get_logger().info(f'Left distance: {left_distance:.2f} meters')
		self.get_logger().info(f'Left 45 distance: {left_45:.2f} meters')

		if forward_distance < 0.2:
			print("Stop")
			self.stop()
		# or (self.cur_dir == "right" and left_distance_ > 2.0 and forward_distance_ < 2.0)
		elif forward_distance < 1.6 and right_15 < 1.6:# and right_distance < 0.5:
			#self.avoid_toggle = 1
			#right_distance = msg.ranges[398]
			print("Left")
			self.go_left()
			#if right_distance > 0.5:
			#	self.avoid_toggle = 0
			#	self.pid_wall_follow(msg)
			#print("sleeping")
			#time.sleep(0.1)
			self.pid_wall_follow(msg, 0, self.wall_dist)
			#self.wall_dist = 1.4
		else:
			print("Wall follow PID")
			if self.wall_dist > 0.7:
				self.wall_dist = self.wall_dist-0.01
			self.pid_wall_follow(msg, 1, self.wall_dist)

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
		input.angle = 0.8
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




	# start of PID implementation
	def pid_wall_follow(self, msg, act, wall_dist):
		#wall_dist = 0.35
		
		df = msg.ranges[0]
		#original true right should be 398
		dr = msg.ranges[398]
		#original left should be 132
		dl = msg.ranges[132]
		
		dr = float('inf')
		for k  in range(398-0, 398+120, 10):
			dr_temp = 0.01
			l = 0.000001
			for i in range(k-5, k+5, 10):
				try:
					if msg.ranges[i] == float('inf'):
						break
				except:
					break
	
				dr_temp = dr_temp + msg.ranges[i]
				l += 1
			dr_temp = dr_temp/l
			if dr_temp < dr:
				dr = dr_temp
			
		#if dr > wall_dist+0.2 and dr > wall_dist+1:
		#	wall_dist = dr*0.95
			
		self.dr_lp = self.dr_lp*0.6+dr*0.4
		if self.dr_lp > 2:
			self.dr_lp = 2
		dr = self.dr_lp
		


		#ds = dl-dr
		ds = wall_dist-dr
		self.get_logger().info(f'Forward distance: {df:.2f} meters')
		self.get_logger().info(f'Right distance: {dr:.2f} meters')
		self.get_logger().info(f'Left distance: {dl:.2f} meters')

		#PID gains
		k_th_p = 1.0
		k_th_d = 0.0
		k_th_i = 0.0

		k_psi_p = 0.8
		k_psi_d = -0.01
		k_psi_i = 0.0

		#P
		if df < self.df_cut:
			th_p = 0.0
			psi_p = 0.0
			self.stop()
		else:
			th_p = 0.69 #df

		'''if dl < self.ds_min or dr < self.ds_min:
			th_p = 0.0
			psi_p = 0.0
		else:
			psi_p = ds
		'''
		psi_p = ds

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
		if act:
			self.act()
		

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
