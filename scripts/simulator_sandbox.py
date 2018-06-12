#!/usr/bin/env python
import rospy
import sys
from time import time
from geometry_msgs.msg import WrenchStamped as WrenchMsg
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Empty as EmptyMsg

from iai_bullet_sim.basic_simulator import BasicSimulator

from blessed import Terminal

class Watchdog(object):
	def __init__(self, timeout=rospy.Duration(0.1)):
		self.last_tick = rospy.Time()
		self.timeout = timeout

	def tick(self, last_tick):
		self.last_tick = last_tick

	def barks(self):
		return rospy.Time.now() - self.last_tick > self.timeout


class JSPublisher(object):
	def __init__(self, multibody, topic_prefix=''):
		self.publisher = rospy.Publisher('{}/joint_state'.format(topic_prefix), JointStateMsg, queue_size=1)
		self.body = multibody

	def update(self):
		new_js = self.body.joint_state()
		msg = JointStateMsg()
		msg.header.stamp = rospy.Time.now()
		for name, state in new_js.items():
			msg.name.append(name)
			msg.position.append(state.position)
			msg.velocity.append(state.velocity)
			msg.effort.append(state.effort)
		self.publisher.publish(msg)

class SensorPublisher(object):
	def __init__(self, multibody, topic_prefix):
		self.publishers = {}
		for sensor in multibody.joint_sensors:
			self.publishers[sensor] = rospy.Publisher('{}/sensors/{}'.format(topic_prefix, sensor), WrenchMsg, queue_size=1)
		self.body = multibody

	def update(self):
		msg = WrenchMsg()
		msg.header.stamp = rospy.Time.now()
		for sensor, state in self.body.get_sensor_states().items():
			msg.header.frame_id = self.body.joints[sensor].linkName
			msg.wrench.force.x  = state.f[0] 
			msg.wrench.force.y  = state.f[1] 
			msg.wrench.force.z  = state.f[2] 
			msg.wrench.torque.x = state.m[0] 
			msg.wrench.torque.y = state.m[1] 
			msg.wrench.torque.z = state.m[2] 
			self.publishers[sensor].publish(msg)


class CommandSubscriber(object):
	def __init__(self, multibody, topic, topic_type=JointStateMsg):
		self.body = multibody
		self.subscriber = rospy.Subscriber(topic, topic_type, callback=self.cmd_callback, queue_size=1)

	def cmd_callback(self, cmd_msg):
		raise (NotImplemented)

	def tick(self):
		pass


class WatchdoggedJointController(CommandSubscriber):
	def __init__(self, multibody, topic, watchdog_timeout):
		super(WatchdoggedJointController, self).__init__(multibody, topic, JointStateMsg)
		self.watchdogs = {}
		self.next_cmd  = {}
		for joint in self.body.joints.keys():
			self.watchdogs[joint] = Watchdog(rospy.Duration(watchdog_timeout))
			self.next_cmd[joint] = 0

class JointPositionController(WatchdoggedJointController):
	def __init__(self, multibody, topic_prefix, watchdog_timeout=0.2):
		super(JointVelocityController, self).__init__(multibody, '{}/commands/joint_positions'.format(topic_prefix), watchdog_timeout)

	def cmd_callback(self, cmd_msg):
		for x in range(len(cmd_msg.name)):
			self.watchdogs[cmd_msg.name[x]].tick(cmd_msg.header.stamp)
			self.next_cmd[cmd_msg.name[x]] = cmd_msg.position[x]

	def tick(self):
		for jname in self.next_cmd.keys():
			if jname in self.watchdogs and self.watchdogs[jname].barks():
				self.next_cmd[jname] = self.body.joint_state()[jname].position # Stop the joint where it is

		self.body.apply_joint_pos_cmds(self.next_cmd)


class JointVelocityController(WatchdoggedJointController):
	def __init__(self, multibody, topic_prefix, watchdog_timeout=0.2):
		super(JointVelocityController, self).__init__(multibody, '{}/commands/joint_velocities'.format(topic_prefix), watchdog_timeout)

	def cmd_callback(self, cmd_msg):
		for x in range(len(cmd_msg.name)):
			self.watchdogs[cmd_msg.name[x]].tick(cmd_msg.header.stamp)
			self.next_cmd[cmd_msg.name[x]] = cmd_msg.velocity[x]

	def tick(self):
		for jname in self.next_cmd.keys():
			if jname in self.watchdogs and self.watchdogs[jname].barks():
				self.next_cmd[jname] = 0

		self.body.apply_joint_vel_cmds(self.next_cmd)


class JointVelocityDeltaContoller(JointVelocityController):
	def __init__(self, multibody, topic_prefix, watchdog_timeout=0.3):
		super(JointVelocityController, self).__init__(multibody, '{}/commands/joint_velocities'.format(topic_prefix), watchdog_timeout)


class SimulatorNode(object):
	def __init__(self, tick_rate=50):
		self.tick_rate = tick_rate
		self.sim = BasicSimulator(self.tick_rate)
		self.sim.init(mode='gui')
		self.js_publishers = {}

		self.load_world(None)


	def load_world(self, world):
		plane = self.sim.load_urdf('package://iai_bullet_sim/urdf/plane.urdf', useFixedBase=1)

		self.ur5 = self.sim.load_urdf('package://iai_table_robot_description/robots/ur5_table.urdf', useFixedBase=1)
		self.ur5.enable_joint_sensor('wrist_3_joint')
		ur5Id = self.sim.get_body_id(self.ur5.bId())
		self.ur5_js_pub = JSPublisher(self.ur5, ur5Id)
		self.ur5_sensor_pub = SensorPublisher(self.ur5, ur5Id)
		self.ur5_cmd_sub = JointVelocityController(self.ur5, ur5Id)
		self.sim.register_pre_physics_cb(self.ur5_cmd_sub.tick)
		self.sim.register_post_physics_cb(self.ur5_js_pub.update)
		self.sim.register_post_physics_cb(self.ur5_sensor_pub.update)


	def tick(self, timer_event):
		self.sim.update()



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

	sn = AFAPSimulator(50)
	tick_sub = rospy.Subscriber('/trigger_tick', EmptyMsg, callback=sn.tick, queue_size=1)

	print('System should have ticked')

	for x in range(200):
		sn.tick(None)

	while not rospy.is_shutdown():
		pass #sn.tick(None)

