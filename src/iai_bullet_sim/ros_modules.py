import rospy
from collections import OrderedDict

from geometry_msgs.msg import WrenchStamped as WrenchMsg
from sensor_msgs.msg import JointState as JointStateMsg
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryMsg

from iai_bullet_sim.basic_simulator import SimulatorPlugin

class Watchdog(object):
	def __init__(self, timeout=rospy.Duration(0.1)):
		self.last_tick = rospy.Time()
		self.timeout = timeout

	def tick(self, last_tick):
		self.last_tick = last_tick

	def barks(self):
		return rospy.Time.now() - self.last_tick > self.timeout


class JSPublisher(SimulatorPlugin):
	def __init__(self, multibody, topic_prefix=''):
		super(JSPublisher, self).__init__('JointState Publisher')
		self.publisher = rospy.Publisher('{}/joint_state'.format(topic_prefix), JointStateMsg, queue_size=1, tcp_nodelay=True)
		self.body = multibody

	def post_physics_update(self, simulator, deltaT):
		new_js = self.body.joint_state()
		msg = JointStateMsg()
		msg.header.stamp = rospy.Time.now()
		for name, state in new_js.items():
			msg.name.append(name)
			msg.position.append(state.position)
			msg.velocity.append(state.velocity)
			msg.effort.append(state.effort)
		self.publisher.publish(msg)


class SensorPublisher(SimulatorPlugin):
	def __init__(self, multibody, topic_prefix):
		super(SensorPublisher, self).__init__('Sensor Publisher')
		self.publishers = {}
		for sensor in multibody.joint_sensors:
			self.publishers[sensor] = rospy.Publisher('{}/sensors/{}'.format(topic_prefix, sensor), WrenchMsg, queue_size=1, tcp_nodelay=True)
		self.body = multibody

	def post_physics_update(self, simulator, deltaT):
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


class CommandSubscriber(SimulatorPlugin):
	def __init__(self, name, multibody, topic, topic_type=JointStateMsg):
		super(CommandSubscriber, self).__init__(name)
		self.body = multibody
		self.subscriber = rospy.Subscriber(topic, topic_type, callback=self.cmd_callback, queue_size=1)

	def cmd_callback(self, cmd_msg):
		raise (NotImplemented)


class WatchdoggedJointController(CommandSubscriber):
	def __init__(self, name, multibody, topic, watchdog_timeout):
		super(WatchdoggedJointController, self).__init__(name, multibody, topic, JointStateMsg)
		self.watchdogs = {}
		self.next_cmd  = {}
		for joint in self.body.joints.keys():
			self.watchdogs[joint] = Watchdog(rospy.Duration(watchdog_timeout))
			self.next_cmd[joint] = 0


class JointPositionController(WatchdoggedJointController):
	def __init__(self, multibody, topic_prefix, watchdog_timeout=0.2):
		super(JointVelocityController, self).__init__('Watchdogged Joint Position Controller', multibody, '{}/commands/joint_positions'.format(topic_prefix), watchdog_timeout)

	def cmd_callback(self, cmd_msg):
		for x in range(len(cmd_msg.name)):
			self.watchdogs[cmd_msg.name[x]].tick(cmd_msg.header.stamp)
			self.next_cmd[cmd_msg.name[x]] = cmd_msg.position[x]

	def pre_physics_update(self, simulator, deltaT):
		for jname in self.next_cmd.keys():
			if jname in self.watchdogs and self.watchdogs[jname].barks():
				self.next_cmd[jname] = self.body.joint_state()[jname].position # Stop the joint where it is

		self.body.apply_joint_pos_cmds(self.next_cmd)


class JointVelocityController(WatchdoggedJointController):
	def __init__(self, multibody, topic_prefix, watchdog_timeout=0.2):
		super(JointVelocityController, self).__init__('Watchdogged Joint Velocity Controller', multibody, '{}/commands/joint_velocities'.format(topic_prefix), watchdog_timeout)

	def cmd_callback(self, cmd_msg):
		for x in range(len(cmd_msg.name)):
			self.watchdogs[cmd_msg.name[x]].tick(cmd_msg.header.stamp)
			self.next_cmd[cmd_msg.name[x]] = cmd_msg.velocity[x]

	def pre_physics_update(self, simulator, deltaT):
		for jname in self.next_cmd.keys():
			if jname in self.watchdogs and self.watchdogs[jname].barks():
				self.next_cmd[jname] = 0

		self.body.apply_joint_vel_cmds(self.next_cmd)


# This controller is supposed to publish the differences between the given commands and the state of the simulation
class JointVelocityDeltaContoller(JointVelocityController):
	def __init__(self, multibody, topic_prefix, watchdog_timeout=0.3):
		super(JointVelocityDeltaController, self).__init__(multibody, topic_prefix, watchdog_timeout)
		raise (NotImplemented)


class JointVelocityTieInController(JointVelocityController):
	def __init__(self, multibody, topic_prefix, watchdog_timeout=0.2):
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
	def __init__(self, multibody, topic_prefix):
		super(TrajectoryPositionController, self).__init__('Trajectory Position Controller', multibody, '{}/commands/joint_trajectory'.format(topic_prefix), JointTrajectoryMsg)
		self.trajectory = None
		self.t_start = None
		self.__t_index = 0

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
		if self.trajectory is None:
			return

		tss = rospy.Time.now() - self.t_start
		if self.trajectory[self.__t_index][0] < tss and self.__t_index < len(self.trajectory) - 1:
			self.__t_index += 1

		self.body.apply_joint_pos_cmds(self.trajectory[self.__t_index][1])
