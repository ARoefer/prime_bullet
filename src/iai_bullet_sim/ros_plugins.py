import rospy
from collections import OrderedDict

from geometry_msgs.msg import WrenchStamped as WrenchMsg
from sensor_msgs.msg import JointState as JointStateMsg
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryMsg

from iai_bullet_sim.basic_simulator import SimulatorPlugin

class Watchdog(object):
	"""Watchdog class. Barks if not ticked for long enough."""
	def __init__(self, timeout=rospy.Duration(0.1)):
		"""Constructor. Sets time after which the dog barks."""
		self.last_tick = rospy.Time()
		self.timeout = timeout

	def tick(self, last_tick):
		"""Sets the last time the dog was ticked."""
		self.last_tick = last_tick

	def barks(self):
		"""Returns True, if last tick was long enough ago."""
		return rospy.Time.now() - self.last_tick > self.timeout


class JSPublisher(SimulatorPlugin):
	"""Plugin which publishes an object's joint state to a topic.
	The joint state will be published to [prefix]/joint_states.
	"""
	def __init__(self, multibody, topic_prefix=''):
		"""Initializes the plugin.

		multibody    -- Object to observe.
		topic_prefix -- Prefix for the topic to publish to.
		"""
		super(JSPublisher, self).__init__('JointState Publisher')
		self.publisher = rospy.Publisher('{}/joint_states'.format(topic_prefix), JointStateMsg, queue_size=1, tcp_nodelay=True)
		self.body = multibody
		self.topic_prefix = topic_prefix
		self.__enabled = True

	def post_physics_update(self, simulator, deltaT):
		"""Publishes the current joint state."""
		if self.__enabled is False:
			return

		new_js = self.body.joint_state()
		msg = JointStateMsg()
		msg.header.stamp = rospy.Time.now()
		for name, state in new_js.items():
			msg.name.append(name)
			msg.position.append(state.position)
			msg.velocity.append(state.velocity)
			msg.effort.append(state.effort)
		self.publisher.publish(msg)

	def disable(self, simulator):
		"""Disables the publisher."""
		self.__enabled = False
		self.publisher.unregister()

	def to_dict(self, simulator):
		"""Serializes this plugin to a dictionary."""
		return {'body': simulator.get_body_id(self.body.bId()),
				'topic_prefix': self.topic_prefix}

	@classmethod
	def factory(cls, simulator, init_dict):
		"""Instantiates the plugin from a dictionary in the context of a simulator."""
		body = simulator.get_body(init_dict['body'])
		if body is None:
			raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
		return SensorPublisher(body, init_dict['topic_prefix'])


class SensorPublisher(SimulatorPlugin):
	"""Plugin which publishes an object's sensor read outs to a topic per sensor.
	The sensor data will be published to [prefix]/sensors/[sensor] as geometry_msgs/WrenchStamped.
	"""
	def __init__(self, multibody, topic_prefix=''):
		"""Initializes the plugin.

		multibody    -- Object to observe.
		topic_prefix -- Prefix for the topics.
		"""
		super(SensorPublisher, self).__init__('Sensor Publisher')
		self.publishers = {}
		for sensor in multibody.joint_sensors:
			self.publishers[sensor] = rospy.Publisher('{}/sensors/{}'.format(topic_prefix, sensor), WrenchMsg, queue_size=1, tcp_nodelay=True)
		self.body = multibody
		self.__topic_prefix = topic_prefix
		self.__enabled = True

	def post_physics_update(self, simulator, deltaT):
		"""Publishes the current sensor states."""
		if self.__enabled is False:
			return

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

	def disable(self, simulator):
		"""Disables the publishers."""
		self.__enabled = False
		for publisher in self.publishers.values():
			publisher.unregister()

	def to_dict(self, simulator):
		"""Serializes this plugin to a dictionary."""
		return {'body': simulator.get_body_id(self.body.bId()),
				'topic_prefix': self.__topic_prefix}

	@classmethod
	def factory(cls, simulator, init_dict):
		"""Instantiates the plugin from a dictionary in the context of a simulator."""
		body = simulator.get_body(init_dict['body'])
		if body is None:
			raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
		return SensorPublisher(body, init_dict['topic_prefix'])


class CommandSubscriber(SimulatorPlugin):
	"""Superclass for plugins which subscribe to a command topic."""
	def __init__(self, name, multibody, topic, topic_type=JointStateMsg):
		"""Initializes the plugin.

		name         -- Name of the plugin.
		multibody    -- Object to observe.
		topic        -- Topic to subscribe to.
		topic_type   -- Message type of the topic.
		"""
		super(CommandSubscriber, self).__init__(name)
		self.body = multibody
		self.subscriber = rospy.Subscriber(topic, topic_type, callback=self.cmd_callback, queue_size=1)
		self._enabled = True

	def cmd_callback(self, cmd_msg):
		"""Implements the command processing behavior."""
		raise (NotImplemented)

	def disable(self, simulator):
		"""Disables the plugin and subscriber."""
		self._enabled = False
		self.subscriber.unregister()


class WatchdoggedJointController(CommandSubscriber):
	"""Superclass for per joint controller.

	"""
	def __init__(self, name, multibody, topic, watchdog_timeout):
		super(WatchdoggedJointController, self).__init__(name, multibody, topic, JointStateMsg)
		self.watchdogs = {}
		self.next_cmd  = {}
		for joint in self.body.joints.keys():
			self.watchdogs[joint] = Watchdog(rospy.Duration(watchdog_timeout))
			self.next_cmd[joint] = 0
		self.watchdog_timeout = watchdog_timeout


class JointPositionController(WatchdoggedJointController):
	def __init__(self, multibody, topic_prefix='', watchdog_timeout=0.2):
		super(JointVelocityController, self).__init__('Watchdogged Joint Position Controller', multibody, '{}/commands/joint_positions'.format(topic_prefix), watchdog_timeout)
		self.__topic_prefix = topic_prefix

	def cmd_callback(self, cmd_msg):
		for x in range(len(cmd_msg.name)):
			self.watchdogs[cmd_msg.name[x]].tick(cmd_msg.header.stamp)
			self.next_cmd[cmd_msg.name[x]] = cmd_msg.position[x]

	def pre_physics_update(self, simulator, deltaT):
		if self._enabled:
			for jname in self.next_cmd.keys():
				if jname in self.watchdogs and self.watchdogs[jname].barks():
					self.next_cmd[jname] = self.body.joint_state()[jname].position # Stop the joint where it is

		self.body.apply_joint_pos_cmds(self.next_cmd)

	def to_dict(self, simulator):
		return {'body': simulator.get_body_id(self.body.bId()),
				'topic_prefix': self.__topic_prefix,
				'watchdog_timeout': self.watchdog_timeout}

	@classmethod
	def factory(cls, simulator, init_dict):
		body = simulator.get_body(init_dict['body'])
		if body is None:
			raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
		return JointPositionController(body, init_dict['topic_prefix'], init_dict['watchdog_timeout'])

class JointVelocityController(WatchdoggedJointController):
	def __init__(self, multibody, topic_prefix='', watchdog_timeout=0.2):
		super(JointVelocityController, self).__init__('Watchdogged Joint Velocity Controller', multibody, '{}/commands/joint_velocities'.format(topic_prefix), watchdog_timeout)
		self.__topic_prefix = topic_prefix

	def cmd_callback(self, cmd_msg):
		for x in range(len(cmd_msg.name)):
			self.watchdogs[cmd_msg.name[x]].tick(cmd_msg.header.stamp)
			self.next_cmd[cmd_msg.name[x]] = cmd_msg.velocity[x]

	def pre_physics_update(self, simulator, deltaT):
		if self._enabled:
			for jname in self.next_cmd.keys():
				if jname in self.watchdogs and self.watchdogs[jname].barks():
					self.next_cmd[jname] = 0

		self.body.apply_joint_vel_cmds(self.next_cmd)

	def to_dict(self, simulator):
		return {'body': simulator.get_body_id(self.body.bId()),
				'topic_prefix': self.__topic_prefix,
				'watchdog_timeout': self.watchdog_timeout}

	@classmethod
	def factory(cls, simulator, init_dict):
		body = simulator.get_body(init_dict['body'])
		if body is None:
			raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
		return JointVelocityController(body, init_dict['topic_prefix'], init_dict['watchdog_timeout'])


# This controller is supposed to publish the differences between the given commands and the state of the simulation
class JointVelocityDeltaContoller(JointVelocityController):
	def __init__(self, multibody, topic_prefix='', watchdog_timeout=0.3):
		super(JointVelocityDeltaController, self).__init__(multibody, topic_prefix, watchdog_timeout)
		raise (NotImplemented)


class JointVelocityTieInController(JointVelocityController):
	def __init__(self, multibody, topic_prefix='', watchdog_timeout=0.2):
		super(JointVelocityTieInController, self).__init__(multibody, topic_prefix, watchdog_timeout)
		self.__callbacks = set()

	def cmd_callback(self, cmd_msg):
		super(JointVelocityTieInController, self).cmd_callback(cmd_msg)
		for cb in self.__callbacks:
			cb(cmd_msg)

	def register_callback(self, callback):
		self.__callbacks.add(callback)

	def deregister_callback(self, callback):
		self.__callbacks.remove(callback)


class TrajectoryPositionController(CommandSubscriber):
	def __init__(self, multibody, topic_prefix=''):
		super(TrajectoryPositionController, self).__init__('Trajectory Position Controller', multibody, '{}/commands/joint_trajectory'.format(topic_prefix), JointTrajectoryMsg)
		self.trajectory = None
		self.t_start = None
		self.__t_index = 0
		self.__topic_prefix = topic_prefix

	def cmd_callback(self, cmd_msg):
		print('Received new trajectory with {} points'.format(len(cmd_msg.points)))
		self.trajectory = []
		for point in cmd_msg.points:
			self.trajectory.append((point.time_from_start
, {cmd_msg.joint_names[x]: point.positions[x] for x in range(len(cmd_msg.joint_names))}))
		self.trajectory = sorted(self.trajectory)
		self.t_start = rospy.Time.now()
		self.__t_index = 0
		self.body.set_joint_positions(self.trajectory[0][1])

	def pre_physics_update(self, simulator, deltaT):
		if self.trajectory is None or self._enabled is False:
			return

		tss = rospy.Time.now() - self.t_start
		if self.trajectory[self.__t_index][0] < tss and self.__t_index < len(self.trajectory) - 1:
			self.__t_index += 1

		self.body.apply_joint_pos_cmds(self.trajectory[self.__t_index][1])

	def to_dict(self, simulator):
		return {'body': simulator.get_body_id(self.body.bId()),
				'topic_prefix': self.__topic_prefix}

	@classmethod
	def factory(cls, simulator, init_dict):
		body = simulator.get_body(init_dict['body'])
		if body is None:
			raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
		return TrajectoryPositionController(body, init_dict['topic_prefix'])


PLUGIN_LIST = [JSPublisher, SensorPublisher, JointPositionController, JointVelocityController, TrajectoryPositionController] 
PLUGINS = {str(p): p for p in PLUGIN_LIST}