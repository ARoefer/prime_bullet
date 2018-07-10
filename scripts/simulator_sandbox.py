#!/usr/bin/env python
import rospy
import sys
import yaml 
from time import time
from rosgraph_msgs.msg import Clock as ClockMsg
from std_msgs.msg import Empty as EmptyMsg

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.ros_plugins import JSPublisher, SensorPublisher, JointVelocityController, TrajectoryPositionController, PLUGINS

from blessed import Terminal


class SimulatorNode(object):
	def __init__(self):
		pass

	def init_from_yaml(self, yaml_path, mode='gui'):
		self.init(yaml.load(open(yaml_path, 'r')))

	def init_from_rosparam(self, rosparam, mode='gui'):
		self.init(rospy.get_param(rosparam, None))

	def init(self, config_dict=None, mode='gui'):
		cdict = config_dict if config_dict is not None else {'tick_rate': 50, 'gravity': [0,0,-9.81]}
		self.sim = BasicSimulator()
		self.sim.init(mode)
		self.sim.load_simulator(cdict, PLUGINS)

	def load_world(self, world):
		plane = self.sim.load_urdf('package://iai_bullet_sim/urdf/plane.urdf', useFixedBase=1)
		kitchen = self.sim.load_urdf('package://iai_kitchen/urdf/IAI_kitchen.urdf', useFixedBase=1)

		# self.ur5 = self.sim.load_urdf('package://iai_table_robot_description/robots/ur5_table.urdf', useFixedBase=1)
		# self.ur5.enable_joint_sensor('wrist_3_joint')
		# ur5Id = self.sim.get_body_id(self.ur5.bId())
		# self.ur5_js_pub = JSPublisher(self.ur5, ur5Id)
		# self.ur5_sensor_pub = SensorPublisher(self.ur5, ur5Id)
		# self.ur5_traj_controller = TrajectoryPositionController(self.ur5, ur5Id)

		sphere   = self.sim.create_sphere(pos=[1,0,3], mass=100, radius=0.15)
		sphere2  = self.sim.create_sphere(pos=[1,0,4], mass=100, radius=0.15)
		# box      = self.sim.create_box(pos=[3,0,3], mass=100)
		# cylinder = self.sim.create_cylinder(pos=[5,0,3], mass=100)
		# capsule  = self.sim.create_capsule(pos=[7,0,3], mass=100)

		# self.sim.register_plugin(self.ur5_js_pub)
		# self.sim.register_plugin(self.ur5_sensor_pub)
		# self.sim.register_plugin(self.ur5_traj_controller)


	def tick(self, timer_event):
		self.sim.update()

	def run(self):
		pass

	def pause(self):
		pass

	def reset(self):
		self.sim.reset()

	def stop(self):
		self.sim.stop()

	def kill(self):
		self.stop()
		self.sim.kill()

	def save_to_yaml(self, yaml_path, use_current_state_as_init=False):
		out = open(yaml_path, 'w')
		out.write(yaml.dump(self.sim.save_simulator(use_current_state_as_init)))
		out.close()

	def save_to_rosparam(self, rosparam, use_current_state_as_init=False):
		rospy.set_param(rosparam, self.sim.save_simulator(use_current_state_as_init))


class FixedTickSimulator(SimulatorNode):
	def __init__(self):
		super(FixedTickSimulator, self).__init__()
		self.timer = None

	def init(self, config_dict=None):
		super(FixedTickSimulator, self).init(config_dict)

	def pause(self):
		super(FixedTickSimulator, self).pause()

		if self.timer is not None:
			self.timer.shutdown()
			self.timer = None

	def run(self):
		super(FixedTickSimulator, self).run()
		if self.timer is None:
			self.timer = rospy.Timer(rospy.Duration(1.0 / self.sim.tick_rate), self.tick)

	def stop(self):
		self.pause()
		super(FixedTickSimulator, self).stop()


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

	sn = FixedTickSimulator()
	#sn.init_from_yaml('quit_current.yaml')
	sn.init()
	sn.load_world(None)
	sn.run()

	while not rospy.is_shutdown():
		pass

	sn.stop()
	#sn.save_to_yaml('quit_initial.yaml')
	#sn.save_to_yaml('quit_current.yaml', True)
	sn.kill()

