import rospy
from geometry_msgs.msg import WrenchStamped as WrenchMsg
from sensor_msgs.msg import JointState as JointStateMsg


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