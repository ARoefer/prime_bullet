import pybullet as pb
from collections import namedtuple
from iai_bullet_sim.utils import Vector3, Quaternion, Frame

class JointDriver(object):
	def update_positions(self, robot_data, positions_dict):
		pass
	def update_velocities(self, robot_data, velocities_dict):
		pass
	def update_effort(self, robot_data, effort_dict):
		pass

class JointState(object):
	def __init__(self, pos, vel, rForce, appliedTorque):
		self.position = pos
		self.velocity = vel
		self.reactionForce = ReactionForces(rForce[:3], rForce[3:])
		self.effort = appliedTorque

ReactionForces  = namedtuple('ReactionForces', ['f', 'm'])

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType', 'qIndex', 'uIndex',
									 'flags', 'jointDamping', 'jointFriction', 'lowerLimit',
									 'upperLimit', 'maxEffort', 'maxVelocity', 'linkName',
									 'axis', 'parentFramePos', 'parentFrameOrn', 'parentIndex'])

LinkState  = namedtuple('LinkState', ['CoMFrame', 'localInertialFrame', 'worldFrame', 'linearVelocity', 'angularVelocity'])


class MultiBody(object):
	def __init__(self, simulator, bulletId, color, initial_pos=[0,0,0], initial_rot=[0,0,0,1], joint_driver=JointDriver(), urdf_file=None):
		self.simulator      = simulator
		self.__bulletId     = bulletId
		self.color          = color
		self.joints         = {}
		self.joint_idx_name_map = {}
		self.joint_sensors  = set()
		self.joint_driver   = joint_driver
		self.link_index_map = {}
		self.urdf_file      = urdf_file

		self.initial_pos    = initial_pos
		self.initial_rot    = initial_rot
		self.initial_joint_state = {}

		multibodyName, base_link = pb.getBodyInfo(self.__bulletId)
		self.base_link = base_link
		self.link_index_map[self.base_link] = -1

		for x in range(pb.getNumJoints(self.__bulletId)):
			joint = JointInfo(*pb.getJointInfo(self.__bulletId, x))
			self.joints[joint.jointName] = joint
			if joint.jointType != pb.JOINT_FIXED:
				self.initial_joint_state[joint.jointName] = min(max(joint.lowerLimit, 0.0), joint.upperLimit)

		
		self.__index_joint_map       = {info.jointIndex: joint for joint, info in self.joints.items()}
		self.__dynamic_joint_indices = [info.jointIndex for info in self.joints.values() if info.jointType != pb.JOINT_FIXED]
		print('dynamic joints:\n  {}'.format('\n  '.join([info.jointName for info in self.joints.values() if info.jointType != pb.JOINT_FIXED])))


		self.links = {info.linkName for info in self.joints.values()}
		self.links.add(self.base_link)
		self.__link_index_map = {info.linkName: info.jointIndex for info in self.joints.values()}
		self.__link_index_map[self.base_link] = -1

		self.__current_pose = None
		self.__last_sim_pose_update = -1

		self.__joint_state = None
		self.__last_sim_js_update = -1


	def bId(self):
		return self.__bulletId

	def reset(self):
		pb.resetBasePositionAndOrientation(self.__bulletId, self.initial_pos, self.initial_rot)
		self.set_joint_positions(self.initial_joint_state)
		self.__last_sim_pose_update = -1
		self.__last_sim_js_update = -1

	def get_link_state(self, link=None):
		if link is not None and link not in self.__link_index_map:
			raise Exception('Link "{}" is not defined'.format(link))

		zero_vector = Vector3(0,0,0)
		if link == None or link == self.base_link:
			pos, quat = pb.getBasePositionAndOrientation(self.__bulletId)
			frame = Frame(Vector3(*pos), Quaternion(*quat))
			return LinkState(frame, frame, frame, zero_vector, zero_vector)
		else:
			ls = pb.getLinkState(self.__bulletId, self.__link_index_map[link], compute_vel)
			return LinkState(Frame(Vector3(*ls[0]), Quaternion(*ls[1])),
							 Frame(Vector3(*ls[2]), Quaternion(*ls[3])),
							 Frame(Vector3(*ls[4]), Quaternion(*ls[5])),
							 zero_vector,
							 zero_vector)

	def get_AABB(self, linkId=None):
		res = pb.getAABB(self.__bulletId, self.__link_index_map[linkId])
		return AABB(Vector3(*res[0]), Vector3(*res[1]))

	def joint_state(self):
		if self.simulator.get_n_update() != self.__last_sim_js_update:
			new_js = [JointState(*x) for x in pb.getJointStates(self.__bulletId, self.__dynamic_joint_indices)]
			self.__joint_state = {self.__index_joint_map[self.__dynamic_joint_indices[x]]: new_js[x] for x in range(len(new_js))}

		return self.__joint_state

	def pose(self):
		if self.simulator.get_n_update() != self.__last_sim_pose_update:
			temp = pb.getBasePositionAndOrientation(self.__bulletId)
			self.__current_pose = Frame(temp[0], temp[1])
		return self.__current_pose

	def enable_joint_sensor(self, joint_name, enable=True):
		pb.enableJointForceTorqueSensor(self.__bulletId, self.joints[joint_name].jointIndex, enable)
		if enable:
			self.joint_sensors.add(joint_name)
		else:
			self.joint_sensors.remove(joint_name)


	def get_sensor_states(self):
		self.joint_state()

		return {sensor: self.__joint_state[sensor].reactionForce for sensor in self.joint_sensors}


	def set_pose(self, pose, override_initial=False):
		pos  = pose.position
		quat = pose.quaternion
		pb.resetBasePositionAndOrientation(self.__bulletId, pos, quat)
		self.__last_sim_pose_update = -1
		if override_initial:
			self.initial_pos = pos
			self.initial_rot = quat


	def set_joint_positions(self, state, override_initial=False):
		for j, p in state.items():
			pb.resetJointState(self.__bulletId, self.joints[j].jointIndex, p)
		self.__last_sim_js_update = -1
		if override_initial:
			self.initial_joint_state.update(state)


	def apply_joint_pos_cmds(self, cmd):
		cmd_indices = []
		cmd_pos     = []
		self.joint_driver.update_positions(self, cmd)
		for jname in cmd.keys():
			cmd_indices.append(self.joints[jname].jointIndex)
			cmd_pos.append(cmd[jname])

		pb.setJointMotorControlArray(self.__bulletId, cmd_indices, pb.POSITION_CONTROL, targetPositions=cmd_pos)


	def apply_joint_vel_cmds(self, cmd):
		cmd_indices = []
		cmd_vels    = []
		self.joint_driver.update_velocities(self, cmd)
		for jname in cmd.keys():
			cmd_indices.append(self.joints[jname].jointIndex)
			cmd_vels.append(cmd[jname])

		pb.setJointMotorControlArray(self.__bulletId, cmd_indices, pb.VELOCITY_CONTROL, targetVelocities=cmd_vels)

	def apply_joint_effort_cmds(self, cmd):
		cmd_indices = []
		cmd_torques = []
		self.joint_driver.update_effort(self, cmd)
		for jname in cmd.keys():
			cmd_indices.append(self.joints[jname].jointIndex)
			cmd_torques.append(cmd[jname])

		pb.setJointMotorControlArray(self.__bulletId, cmd_indices, pb.TORQUE_CONTROL, forces=cmd_torques)