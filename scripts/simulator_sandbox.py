#!/usr/bin/env python
import rospy
import sys
import yaml 
from time import time
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import Empty as EmptyMsg

from iai_bullet_sim.basic_simulator_node import BasicSimulatorNode
from iai_bullet_sim.ros_plugins import JSPublisher, SensorPublisher, JointVelocityController, TrajectoryPositionController, PLUGINS

from blessed import Terminal


class AFAPSimulator(BasicSimulatorNode):
	def __init__(self, tick_rate):
		super(AFAPSimulator, self).__init__(tick_rate)
		try:
			use_sim_time = rospy.get_param('/use_sim_time')
			if not use_sim_time:
				raise KeyError
		except KeyError:
			raise Exception('Parameter /use_sim_time must be set to "True" for this simulator to work!')

		self.clock_pub = rospy.Publisher('/clock', ClockMsg, queue_size=1)
		self.time = rospy.Time.now()
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

	sn = BasicSimulatorNode()
	sn.init()

	sim = sn.sim
	robot = sim.load_urdf('package://iai_table_robot_description/robots/ur5_table.urdf')
	robot.enable_joint_sensor('wrist_3_joint')

	box = sim.create_cylinder(radius=0.01, height=0.005, pos=[-0.3, -0.3, 0.61], mass=0)

	sim.register_plugin(JSPublisher(robot))
	sim.register_plugin(SensorPublisher(robot))
	sim.register_plugin(TrajectoryPositionController(robot))

	while not rospy.is_shutdown():
		pass

	sn.stop()
	sn.save_to_yaml('package://pbsbtest/config/ur5_bolt_config.yaml')
	#sn.save_to_yaml('quit_current.yaml', True)
	sn.kill()

