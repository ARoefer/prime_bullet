#!/usr/bin/env python
import rospy

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.ros_plugins     import JSPublisher, SensorPublisher, JointVelocityController

from iai_naive_kinematics_sim.msg import ProjectionClock as ProjectionClockMsg
from iai_naive_kinematics_sim.srv import SetJointState as SetJointStateSrv

from rosgraph_msgs.msg import Clock as ClockMsg


class Node(object):
	def __init__(self):
		temp = open('temp.urdf', 'w')
		temp.write(rospy.get_param('robot_description'))
		temp.close()

		self.sim = BasicSimulator()
		self.sim.init()
		self.sim.set_tick_rate(rospy.get_param('~sim_frequency', 50))
		self.watchdog_period = rospy.get_param('~watchdog_period', 0.1)
		self.projection_mode = rospy.get_param('~projection_mode', False)

		self.robot = self.sim.load_urdf('temp.urdf', useFixedBase=True)
		self.robot.set_joint_positions(rospy.get_param('~start_config', {}))

		self.sim.register_plugin(JSPublisher(self.robot))
		self.sim.register_plugin(SensorPublisher(self.robot))
		self.sim.register_plugin(JointVelocityController(self.robot, watchdog_timeout=self.watchdog_period))

		self.srv_server_set_js = rospy.Service('~set_joint_states', SetJointStateSrv, self.srv_set_js) 

		if self.projection_mode:
			try:
				use_sim_time = rospy.get_param('/use_sim_time')
				if not use_sim_time:
					raise KeyError

				self.clock_pub = rospy.Publisher('/clock', ClockMsg, queue_size=1, tcp_nodelay=True)
				self.time = rospy.Time.now()
				self.time_step = rospy.Duration(1.0 / self.tick_rate)
				self.proj_sub  = rospy.Subscriber('~/projection_command', ProjectionClockMsg, callback=self.proj_callback, queue_size=1)
			except KeyError:
				raise Exception('Parameter /use_sim_time must be set to "True" use projection mode!')
		else:
			self.timer = rospy.Timer(rospy.Duration(1.0 / self.sim.tick_rate), self.tick)

	def tick(self, timer_event):
		self.sim.update()

	def tick_and_tock(self):
		self.sim.pre_update()
		self.sim.physics_update()
		self.time += self.time_step
		cmsg = ClockMsg()
		cmsg.clock = self.time
		self.clock_pub.publish(cmsg)
		self.sim.post_update()


	def srv_set_js(self, request):
		js_msg = request.state
		js = {js_msg.name[x]: js_msg.position[x] for x in range(len(js_msg.name))}
		for j in js.keys():
			if j not in self.robot.joints:
				return SetJointStateSrv.Response(False, 'Can not set position for joint {}, because it is not part of the controlled robot.'.format(j))

		self.robot.set_joint_positions(js, True)
		self.sim.reset()
		return SetJointStateSrv.Response(True, '')


	def proj_callback(self, msg):
		self.time = copy(msg.time)
		while True:
			self.tick_and_tock()
			if rospy.Time.now() - msg.time >= msg.period:
				break



if __name__ == '__main__':
	rospy.init_node('iai_bullet_sim')

	n = Node()

	while not rospy.is_shutdown():
		pass