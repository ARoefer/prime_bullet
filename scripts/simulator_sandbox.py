#!/usr/bin/env python
import rospy
import sys
from time import time
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import Empty as EmptyMsg

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.ros_modules import JSPublisher, SensorPublisher, JointVelocityController, JointVelocityTieInController, TrajectoryPositionController

from blessed import Terminal

class SimulatorNode(object):
	def __init__(self, tick_rate=50):
		self.tick_rate = tick_rate
		self.sim = BasicSimulator(self.tick_rate)
		self.sim.init(mode='gui')
		self.js_publishers = {}

		self.load_world(None)

	def init(self, config_dictionary):


	def load_world(self, world):
		plane = self.sim.load_urdf('package://iai_bullet_sim/urdf/plane.urdf', useFixedBase=1)

		self.ur5 = self.sim.load_urdf('package://iai_table_robot_description/robots/ur5_table.urdf', useFixedBase=1)
		self.ur5.enable_joint_sensor('wrist_3_joint')
		ur5Id = self.sim.get_body_id(self.ur5.bId())
		self.ur5_js_pub = JSPublisher(self.ur5, ur5Id)
		self.ur5_sensor_pub = SensorPublisher(self.ur5, ur5Id)
		self.ur5_traj_controller = TrajectoryPositionController(self.ur5, ur5Id)
		self.sim.register_plugin(self.ur5_js_pub)
		self.sim.register_plugin(self.ur5_sensor_pub)
		self.sim.register_plugin(self.ur5_traj_controller)


	def tick(self, timer_event):
		self.sim.update()

	def to_dict(self, ):


class FixedTickSimulator(SimulatorNode):
	def __init__(self, tick_rate):
		super(FixedTickSimulator, self).__init__(tick_rate)

		self.timer = rospy.Timer(rospy.Duration(1.0 / self.tick_rate), self.tick)


class AFAPSimulator(SimulatorNode):
	def __init__(self, tick_rate):
		super(AFAPSimulator, self).__init__(tick_rate)
		try:
			use_sim_time = rospy.get_param('/use_sim_time')
			if not use_sim_time:
				raise KeyError
		except KeyError:
			raise Exception('Parameter /use_sim_time must be set to "True" for this simulator to work!')

		self.clock_pub = rospy.Publisher('/clock', ClockMsg, queue_size=1)
		self.time = rospy.Time()
		self.time_step = rospy.Duration(1.0 / self.tick_rate)
		self.terminal = Terminal()
		self.__last_tick = None
		self.__last_msg_len = 0

	def tick(self, timer_event):
		self.time += self.time_step
		cmsg = ClockMsg()
		cmsg.clock = self.time
		self.clock_pub.publish(cmsg)
		super(AFAPSimulator, self).tick(None)
		now = time()
		if self.__last_tick != None:
			deltaT = now - self.__last_tick
			ratio  = self.time_step.to_sec() / deltaT
			sys.stdout.write(self.terminal.move(self.terminal.height,  0))
			msg = u'Simulation Factor: {}'.format(ratio)
			self.__last_msg_len = len(msg)
			sys.stdout.write(msg)
			sys.stdout.flush()
		self.__last_tick = now



if __name__ == '__main__':
	rospy.init_node('iai_bullet_sim')

	sn = FixedTickSimulator(50)

	while not rospy.is_shutdown():
		pass #sn.tick(None)

