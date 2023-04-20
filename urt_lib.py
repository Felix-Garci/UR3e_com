import time
import threading
import sys
sys.path.append('..')   
import logging
import time
import rtde_config as rtde_config
import rtde as rtde
import keyboard
from math import sqrt

class urt_lib:
	def __init__(self):
		self.STATE=0
		self.t = threading.Thread(target=self.monitor)
		
		self.ROBOT_HOST = '10.0.0.150' # ip in settings in the tablet
		self.ROBOT_PORT = 30004
		self.config_filename = './examples/control_loop_configuration.xml'
		
		self.con = None
		self.gain = 1
		self.setp = None
		self.watchdog = None

		self.angles = None
		self.position = None
		self.target_ang = None
		self.target_pos = None

		self.check = 0
		self.error = 0.01


	def monitor(self):
		print("monitoring...")
		if not self.con.send_start():
			print("Error conenction")
			sys.exit()

		init_time = time.time()
		while self.STATE != -1:

			state = self.con.receive()
			if state is None:
				break
			joint_angles = state.actual_q
			position = state.actual_TCP_pose
			# Check if the program is running in the Polyscope
			if state.output_int_register_0 != 0:

				if self.STATE == 1:

					self.angles = state.actual_q
					self.position = state.actual_TCP_pose
					
					# Compute control error    
					error = self.compute_error(self.target_ang, state.actual_q)
					# Compute control effort
					control_effort = self.compute_control_effort(error, self.gain)
					# Reformat control effort list into setpoint
					control_effort.append(self.STATE)
					self.list_to_setp(self.setp, control_effort)
					# Send new control effort
					self.con.send(self.setp)
					self.check = self.check_dif(self.target_ang,self.angles)
					if self.check == 1:
						self.STATE = 0     
					

				if self.STATE == 2:

					self.angles = state.actual_q
					self.position = state.actual_TCP_pose

					self.target_pos.append(self.STATE)
					self.list_to_setp(self.setp, self.target_pos)#control_effort
					# Send new control effort        
					self.con.send(self.setp)

					self.check = self.check_dif(self.target_pos,self.position)
					if self.check == 1:
						self.STATE = 0


				if self.STATE == 0:
					self.angles = state.actual_q
					self.position = state.actual_TCP_pose
					self.list_to_setp(self.setp, [0,0,0,0,0,0,0])
					self.con.send(self.setp)

			self.con.send(self.watchdog)
				
	def compute_error(self,target, joints):
		"""Computes a 6D vector containing the error in every joint in the control loop

		Args:
			target (list): List of floats containing the target joint angles in radians
			joints (list): List of floats containing the measured joint angles in radians

		Returns:
			list: List of floats containing the angles error in radians
		"""
		return [j - joints[i] for i,j in enumerate(target)]

	def compute_control_effort(self,error, gain):
		"""Computes a 6D vector containing the control effort in every joint in the control loop

		Args:
			error (list): List of floats containing the angles error in radians
			gain (float): Gain in the control loop (each joint angle error will be multiplied times this value to compute control effort)

		Returns:
			liat: List of floats containing the control efforts in each joint
		"""
		return [i*gain for i in error]

	def list_to_degrees(self,angles):
		"""Converts input list values from radians to degrees

		Args:
			angles (list): List containing angles in radians

		Returns:
			list: List containing angles in degrees
		"""
		return [i*360/(2*3.14592) for i in angles]

	def list_to_radians(self,angles):
		"""Converts input list values from degrees to radians

		Args:
			angles (list): List containing angles in degrees

		Returns:
			list: List containing angles in radians
		"""
		return [i*(2*3.14592)/(360) for i in angles]

	# Reformat setpoint into list
	def setp_to_list(self,setp):
		list = []
		for i in range(0,7):
			list.append(setp.__dict__["input_double_register_%i" % i])
		return list

	# Reformat list into setpoint
	def list_to_setp(self,setp, list):
		for i in range (0,7):
			setp.__dict__["input_double_register_%i" % i] = list[i]
		return setp

	def conect(self):
		logging.getLogger().setLevel(logging.INFO)
		#configuration files
		conf = rtde_config.ConfigFile(self.config_filename)
		state_names, state_types = conf.get_recipe('state')
		setp_names, setp_types = conf.get_recipe('setp')
		watchdog_names, watchdog_types = conf.get_recipe('watchdog')

		#connection to the robot
		self.con = rtde.RTDE(self.ROBOT_HOST, self.ROBOT_PORT)
		self.con.connect()
		connection_state = self.con.connect()

		#check connection
		while connection_state != 0:
			time.sleep(5)
			connection_state = self.con.connect()

		self.con.send_output_setup(state_names, state_types)
		self.setp = self.con.send_input_setup(setp_names, setp_types)
		self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)

		# Initialize 6 registers which will hold the target angle values
		self.setp.input_double_register_0 = 0.0
		self.setp.input_double_register_1 = 0.0
		self.setp.input_double_register_2 = 0.0
		self.setp.input_double_register_3 = 0.0
		self.setp.input_double_register_4 = 0.0
		self.setp.input_double_register_5 = 0.0
		self.setp.input_double_register_6 = 0.0
		  
		# The function "rtde_set_watchdog" in the "rtde_control_loop.urp" creates a 1 Hz watchdog
		self.watchdog.input_int_register_0 = 0
		print("connected")
		self.t.start()
		return 1

	def change(self,n):
		self.STATE = n
		return 0

	def check_dif(self,targ,actual):
		ch = 0
		for i in range(len(actual)):
			if (targ[i]-actual[i])**2 < self.error**2 :
				ch = 1
			else :
				ch = 0
				break
		return ch

	def go_pos(self,position):
		self.check  = 0
		self.target_pos = position
		self.STATE = 2
	def go_ang(self,angles):
		self.check  = 0
		self.target_ang = angles
		self.STATE = 1
	def get_ang(self):
		return self.angles
	def get_pos(self):
		return self.position

	def stop(self):
		self.STATE = -1
		self.t.join()
		self.list_to_setp(self.setp, [0,0,0,0,0,0,0])
		self.con.send(self.setp)
		self.con.disconnect()

		
		return 0


t_a = [1.8097728490829468, -2.384428163568014, -1.1529645919799805, -1.0296293062022706, -1.1461346785174769, 1.0089755058288574]
t_p = [0.05932310678649848, 0.4731251666537511, 0.33391676240638934, -0.3848237463763162, 0.33645474834463723, -1.8293701086343745]

t_a = [3.033494795090519e-05, -1.570799013177389, 1.57079, -1.5707946580699463, 1.570799013177389, -4.941621889287262e-05]
t_p = [-0.2992506459308157, -0.13037277698957006, 0.6069772479575763, 0.785, 0.785, 0]


ur = urt_lib()
ur.conect()

#time.sleep(5)

print("started")


ur.go_pos(t_p)
while ur.check == 0:
	time.sleep(0.01)
print("1")
time.sleep(5)
ur.go_ang(t_a)
while ur.check == 0:
	time.sleep(0.01)

ur.stop()

