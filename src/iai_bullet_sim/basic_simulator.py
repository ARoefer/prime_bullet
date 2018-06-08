import pybullet as pb
import random
from collections import namedtuple
from iai_bullet_sim.utils import res_pkg_path, rot3_to_quat
from pprint import pprint

class JointDriver(object):
	def update_positions(self, robot_data, positions_dict):
		pass
	def update_velocities(self, robot_data, velocities_dict):
		pass
	def update_effort(self, robot_data, effort_dict):
		pass

class MultiBody(object):
	def __init__(self, simulator, bulletId, color, joints={}, initial_pos=[0,0,0], initial_rot=[0,0,0,1], initial_joint_state=None, joint_driver=JointDriver()):
		self.simulator      = simulator
		self.bulletId       = bulletId
		self.color          = color
		self.joints         = joints
		
		self.initial_pos    = initial_pos
		self.initial_rot    = initial_rot
		self.initial_joint_state = initial_joint_state if initial_joint_state != None else {joint: 0.0 for joint, info in self.joints.items() if info.type != pb.JOINT_FIXED}
		
		self.joint_driver   = joint_driver
		
		self.__index_joint_map      = {info.jointIndex: joint for joint, info in self.joints.items()}
		self.__index_joint_map[-1]  = None
		
		self.__link_index_map = {info.linkName: info.jointIndex for info in self.joints.values()}
		self.__link_index_map[None] = -1
		
		self.__index_link_map = {i: ln for ln, i in self.link_index_map.items()}
		self.__joint_sensors = set()

	def get_link_state(self, link=None):
		if link not in self.link_index_map:
			raise Exception('Link "{}" is not defined'.format(link))

		zero_vector = Vector3(0,0,0)
		if link == None:
			pos, quat = pb.getBasePositionAndOrientation(self.bulletId)
			frame = Frame(Vector3(*pos), Quaternion(*quat))
			return LinkState(frame, frame, frame, zero_vector, zero_vector)
		else:
			ls = pb.getLinkState(self.bulletId, self.link_index_map[link], compute_vel)
			return LinkState(Frame(Vector3(*ls[0]), Quaternion(*ls[1])),
							 Frame(Vector3(*ls[2]), Quaternion(*ls[3])),
							 Frame(Vector3(*ls[4]), Quaternion(*ls[5])),
							 zero_vector,
							 zero_vector)

	def get_joint_state(self):



Vector3 = namedtuple('Vector3', ['x', 'y', 'z'])
Quaternion = namedtuple('Quaternion', ['x', 'y', 'z', 'w'])
Frame  = namedtuple('Frame', ['position', 'quaterion'])
ReactionForces  = namedtuple('ReactionForces', ['f', 'm'])

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType', 'qIndex', 'uIndex',
									 'flags', 'jointDamping', 'jointFriction', 'jointLowerLimit',
									 'jointUpperLimit', 'jointMaxForce', 'jointMaxVelocity', 'linkName',
									 'jointAxis', 'parentFramePos', 'parentFrameOrn', 'parentIndex'])

JointState = namedtuple('JointState', ['position', 'velocity', 'reactionForce', 'appliedMotorTorque'])

LinkState  = namedtuple('LinkState', ['CoMFrame', 'localInertialFrame', 'worldFrame', 'linearVelocity', 'angularVelocity'])

Constraint = namedtuple('Constraint', ['bulletId', 'bodyParent', 'bodyChild', 'linkParent', 'linkChild',
						  			   'jointType', 'jointAxis', 'parentJointPosition', 'childJointPosition',
						  			   'parentJointOrientation', 'childJointOrientation'])

VisualShape = namedtuple('VisualShape', ['bulletId', 'linkIndex', 'geometryType', 'dimensions', 'filename', 'localPosition', 'localOrientation', 'rgba'])
CollisionShape = namedtuple('CollisionShape', ['bulletId', 'linkIndex', 'geometryType', 'dimensions', 'filename', 'localPosition', 'localOrientation'])

AABB = namedtuple('AABB', ['min', 'max'])


def vec_add(a, b):
	return Vector3(a[0] + b[0], a[1] + b[1], a[2] + b[2])

def vec_sub(a, b):
	return Vector3(a[0] - b[0], a[1] - b[1], a[2] - b[2])

def vec_scale(a, x):
	return Vector3(a.x * x, a.y * x, a.z * x)

def invert_transform(frame_tuple):
	temp = pb.invertTransform(list(frame_tuple.position), list(frame_tuple.quaterion))
	return Frame(Vector3(*temp[0]), Quaternion(*temp[1]))

def frame_tuple_to_sym_frame(frame_tuple):
	return frame3_quaternion(frame_tuple.quaterion.x,
							 frame_tuple.quaterion.y,
							 frame_tuple.quaterion.z,
							 frame_tuple.quaterion.w,
							 point3(*frame_tuple.position))

class ContactPoint(object):
	def __init__(self, bodyA, bodyB, linkA, linkB, posOnA, posOnB, normalOnB, dist, normalForce):
		self.bodyA = bodyA
		self.bodyB = bodyB
		self.linkA = linkA
		self.linkB = linkB
		self.posOnA = posOnA
		self.posOnB = posOnB
		self.normalOnB = normalOnB
		self.dist = dist
		self.normalForce = normalForce

	def __leq__(self, other):
		return self.dist <= other.dist


def hsva_to_rgba(h, s, v, a):
	h_i = int(round(h*6))
	f = h*6 - h_i
	p = v * (1 - s)
	q = v * (1 - f*s)
	t = v * (1 - (1 - f) * s)
	if h_i==0:
		return [v, t, p, a]
	elif h_i==1:
		return [q, v, p, a]
	elif h_i==2:
		return [p, v, t, a]
	elif h_i==3:
		return [p, q, v, a]
	elif h_i==4:
		return [t, p, v, a]
	elif h_i==5:
		return [v, p, q, a]
	print('h_i is {}'.format(h_i))
	return [1,1,1,a]

def vec3_to_list(vec):
	return [vec[0], vec[1], vec[2]]



class FetchDriver(JointDriver):
	def update_positions(self, robot_data, positions_dict):
		if 'gripper_joint' in positions_dict:
			gstate = positions_dict['gripper_joint']
			positions_dict['r_gripper_finger_joint'] = 0.5 * gstate
			positions_dict['l_gripper_finger_joint'] = 0.5 * gstate
	def update_velocities(self, robot_data, velocities_dict):
		if 'gripper_joint' in velocities_dict:
			gstate = velocities_dict['gripper_joint']
			velocities_dict['r_gripper_finger_joint'] = 0.5 * gstate
			velocities_dict['l_gripper_finger_joint'] = 0.5 * gstate

	def update_effort(self, robot_data, effort_dict):
		if 'gripper_joint' in effort_dict:
			gstate = effort_dict['gripper_joint']
			effort_dict['r_gripper_finger_joint'] = 0.5 * gstate
			effort_dict['l_gripper_finger_joint'] = 0.5 * gstate


class BulletSimulator(object):
	def __init__(self, tick_rate):
		self.bodyIds   = set()
		self.bulletIds_to_bodyIds = {}
		self.bodies    = {}
		self.constraints = {}
		self.watchdogs = {}
		self.tick_rate = tick_rate
		self.time_step = 1.0 / self.tick_rate

		self.__h = random.random()
		self.__nextId = 0
		self.__joint_types = {'FIXED': pb.JOINT_FIXED, 'PRISMATIC': pb.JOINT_PRISMATIC, 'HINGE': pb.JOINT_POINT2POINT}

	def __gen_next_color(self):
		self.__h += 0.618033988749895
		self.__h %= 1.0
		return hsva_to_rgba(self.__h, 0.7, 0.95, 1.0)

	def init(self, gravity=[0,0,-9.81], mode=pb.DIRECT):
		self.physicsClient = pb.connect(mode)#or p.DIRECT for non-graphical version
		pb.setGravity(*gravity)
		pb.setTimeStep(self.time_step)

	def kill(self):
		pb.disconnect()

	def apply_joint_vel_cmds(self, bodyId, next_cmd):
		cmd_indices = []
		cmd_vels    = []
		robot_data  = self.bodies[bodyId]
		robot_data.joint_driver.update_velocities(robot_data, next_cmd)
		for jname in next_cmd.keys():
			cmd_indices.append(robot_data.joints[jname].jointIndex)
			cmd_vels.append(next_cmd[jname])

		pb.setJointMotorControlArray(robot_data.bulletId, cmd_indices, pb.VELOCITY_CONTROL, targetVelocities=cmd_vels)

	# @profile
	def set_joint_positions(self, bodyId, state):
		body_data = self.bodies[bodyId]
		for j, p in state.items():
			pb.resetJointState(body_data.bulletId, body_data.joints[j].jointIndex, p)

	def update(self):
		pb.stepSimulation()

	def set_real_time(self, realTime=True):
		pb.setRealTimeSimulation(realTime)

	def enable_joint_sensor(self, bodyId, joint_name, enable=True):
		body = self.bodies[bodyId]
		pb.enableJointForceTorqueSensor(body.bulletId, body.joints[joint_name].jointIndex, enable)
		if enable:
			body.joint_sensors.add(joint_name)
		else:
			body.joint_sensors.remove(joint_name)

	def get_joint_state(self, bodyId, sim_indices=None):
		body_data   = self.bodies[bodyId]
		if sim_indices == None:
			sim_indices = [j.jointIndex for k, j in body_data.joints.items() if k != None and j.jointType != pb.JOINT_FIXED]
		new_js = [JointState(*x) for x in pb.getJointStates(body_data.bulletId, sim_indices)]
		return {body_data.index_joint_map[sim_indices[x]]: new_js[x] for x in range(len(new_js))}

	def get_sensor_state(self, bodyId):
		body_data   = self.bodies[bodyId]
		sim_indices = {body_data.joints[j].jointIndex: j for j in body_data.joint_sensors}
		new_ss = [ReactionForces(x[:3], x[3:]) for x in pb.getJointStates(body_data.bulletId, sim_indices)]
		return {sim_indices[x]: new_ss[x] for x in range(len(new_ss))}

	def reset(self):
		for bodyId in self.bodies.keys():
			self.reset_body(bodyId)

	def reset_body(self, bodyId):
		body_data = self.bodies[bodyId]
		pb.resetBasePositionAndOrientation(body_data.bulletId, body_data.initial_pos, body_data.initial_rot)
		self.set_joint_positions(bodyId, body_data.initial_joint_state)

	def load_robot(self, urdf_path, pos=[0,0,0], rot=[0,0,0,1], joint_driver=JointDriver()):
		res_urdf_path = res_pkg_path(urdf_path, self.resolver)
		print('Simulator: {}'.format(res_urdf_path))
		robotBId = pb.loadURDF(res_urdf_path,
							 pos,
							 rot,
							 useFixedBase=0,
							 flags=pb.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT) #
		base_link, robotId = pb.getBodyInfo(robotBId)
		self.bodyIds.add(robotId)
		if robotId == 'fetch' and type(joint_driver) == JointDriver:
			joint_driver = FetchDriver()
		joint_map = {None: JointInfo(-1, None, None, None, None,
									 None, None, None, None,
									 None, None, None, base_link,
									 None, None, None, None)}
		joint_idx_name_map = {-1: None}
		joint_initial = {}
		for x in range(pb.getNumJoints(robotBId)):
			joint = JointInfo(*pb.getJointInfo(robotBId, x))
			joint_idx_name_map[x] = joint.jointName
			joint_map[joint.jointName] = joint
			if joint.jointType != pb.JOINT_FIXED:
				joint_initial[joint.jointName] = 0.0

		self.bodies[robotId] = MultiBody(self, robotBId, self.__gen_next_color(), joint_map, pos, rot, joint_initial, joint_driver)
		self.bulletIds_to_bodyIds[robotBId] = robotId
		print('created new robot with id {}'.format(robotId))
		return robotId

	def set_object_pose(self, bodyId, pose, override_initial=False):
		pos  = vec3_to_list(pos_of(pose))
		quat = list(rot3_to_quat(pose))
		pb.resetBasePositionAndOrientation(self.bodies[bodyId].bulletId, pos, quat)
		if override_initial:
			self.bodies[bodyId].initial_pos = pos
			self.bodies[bodyId].initial_rot = quat

	def add_object(self, dl_object):
		color = self.__gen_next_color()
		if DLCompoundObject.is_a(dl_object):
			raise Exception('Compund objects are not supported at this time.')
		elif DLRigidObject.is_a(dl_object):
			Id = ''
			if DLIded.is_a(dl_object):
				Id = dl_object.id
			else:
				Id = 'body_{}'.format(self.__nextId)
				self.__nextId += 1
			if Id in self.bodies:
				self.set_object_pose(Id, dl_object.pose, True)
				return Id

			cId = self.__create_vc_shapes(dl_object, dl_object.pose, color)#, vId
			pos = vec3_to_list(pos_of(dl_object.pose))
			rot = list(rot3_to_quat(dl_object.pose))
			objBId = pb.createMultiBody(dl_object.mass, cId, -1, pos, rot)

			obj_data = BodyData(objBId, color, {None: JointInfo(-1, None, None, None, None,
									                            None, None, None, None,
								    	                        None, None, None, Id,
									                            None, None, None, None)},
								pos, rot, {}, None)
			self.bodyIds.add(Id)
			self.bodies[Id] = obj_data
			self.bulletIds_to_bodyIds[objBId] = Id
			return Id
		raise Exception('Cannot generate Bullet-body for object which is not rigid. Object: {}'.format(str(dl_object)))

	def create_constraint(self, constraintId, parentBody, childBody,
						  parentLink=None, childLink=None,
						  jointType='FIXED', jointAxis=[1,0,0],
						  parentJointPosition=[0,0,0], childJointPosition=[0,0,0],
						  parentJointOrientation=[0,0,0,1], childJointOrientation=[0,0,0,1]):
		if constraintId not in self.constraints:
			parent = self.bodies[parentBody]
			child  = self.bodies[childBody]
			type   = self.__joint_types[jointType]
			parentLinkId = parent.link_index_map[parentLink]
			childLinkId  = child.link_index_map[childLink]
			axis = vec3_to_list(jointAxis)
			pjp = vec3_to_list(parentJointPosition)
			cjp = vec3_to_list(childJointPosition)
			pjo = parentJointOrientation
			cjo = childJointOrientation
			bulletId = pb.createConstraint(parent.bulletId, parentLinkId, child.bulletId,
										   childLinkId, type, axis, pjp, cjp, pjo, cjo)
			self.constraints[constraintId] = Constraint(bulletId, parent, child, parentLink, childLink,
						  			   					type, axis, pjp, cjp, pjo, cjo)
		else:
			raise (NotImplementedError)


	# @profile
	def get_body_pos_rot(self, bodyId):
		return pb.getBasePositionAndOrientation(self.bodies[bodyId].bulletId)

	# @profile
	def get_all_pos_rot(self):
		return {Id: self.get_body_pos_rot(Id) for Id in self.bodyIds}

	# @profile
	def get_body_frame(self, bodyId):
		lpos, lrot = self.get_body_pos_rot(bodyId)
		return frame3_quaternion(lrot[0], lrot[1], lrot[2], lrot[3], point3(*lpos))

	# @profile
	def get_all_body_frames(self):
		return {Id: self.get_body_frame(Id) for Id in self.bodies}

	# @profile
	def get_link_state(self, bodyId, link, compute_vel=0):
		zero_vector = Vector3(0,0,0)
		if link == None:
			pos, quat = pb.getBasePositionAndOrientation(self.bodies[bodyId].bulletId)
			frame = Frame(Vector3(*pos), Quaternion(*quat))
			return LinkState(frame, frame, frame, zero_vector, zero_vector)
		else:
			ls = pb.getLinkState(self.bodies[bodyId].bulletId, self.bodies[bodyId].link_index_map[link], compute_vel)
			return LinkState(Frame(Vector3(*ls[0]), Quaternion(*ls[1])),
							 Frame(Vector3(*ls[2]), Quaternion(*ls[3])),
							 Frame(Vector3(*ls[4]), Quaternion(*ls[5])),
							 zero_vector,
							 zero_vector)

	# @profile
	def get_AABB(self, bodyId, linkId=None):
		res = pb.getAABB(self.bodies[bodyId].bulletId, self.bodies[bodyId].link_index_map[linkId])
		return AABB(Vector3(*res[0]), Vector3(*res[1]))

	# @profile
	def get_overlapping(self, aabb, filter=set()):
		raw_overlap = pb.getOverlappingObjects(vec3_to_list(aabb.min), vec3_to_list(aabb.max))
		if raw_overlap == None:
			return []

		return [(self.bulletIds_to_bodyIds[bulletId], self.bodies[self.bulletIds_to_bodyIds[bulletId]].index_joint_map[linkIdx]) for bulletId, linkIdx in raw_overlap if self.bulletIds_to_bodyIds[bulletId] not in filter]

	# @profile
	def get_contacts(self, bodyA=None, bodyB=None, linkA=None, linkB=None):
		bulletA = self.bodies[bodyA].bulletId if bodyA != None else -1
		bulletB = self.bodies[bodyB].bulletId if bodyB != None else -1
		bulletLA = self.bodies[bodyA].link_index_map[linkA] if bodyA != None and linkA != None else -1
		bulletLB = self.bodies[bodyB].link_index_map[linkB] if bodyB != None and linkB != None else -1
		contacts = []
		if bulletLA == -1 and bulletLB == -1:
			contacts = pb.getContactPoints(bulletA, bulletB)
		elif bulletLA != -1 and bulletLB == -1:
			contacts = pb.getContactPoints(bulletA, bulletB, linkIndexA=bulletLA)
		elif bulletLA == -1 and bulletLB != -1:
			contacts = pb.getContactPoints(bulletA, bulletB, linkIndexB=bulletLB)
		else:
			contacts = pb.getContactPoints(bulletA, bulletB, bulletLA, bulletLB)
		return [ContactPoint(self.bulletIds_to_bodyIds[c[1]],                                   # Body A
							 self.bulletIds_to_bodyIds[c[2]],                                   # Body B
							 self.bodies[self.bulletIds_to_bodyIds[c[1]]].index_link_map[c[3]], # Link of A
							 self.bodies[self.bulletIds_to_bodyIds[c[2]]].index_link_map[c[4]], # Link of B
							 Vector3(*c[5]),                                                # Point on A
							 Vector3(*c[6]),                                                # Point on B
							 Vector3(*c[7]),                                                # Normal from B to A
							 c[8],                                                              # Distance
							 c[9])                                                              # Normal force
				for c in contacts]

	# @profile
	def get_closest_points(self, bodyA, bodyB, linkA=None, linkB=None, dist=0.2):
		bulletA = self.bodies[bodyA].bulletId
		bulletB = self.bodies[bodyB].bulletId
		bulletLA = self.bodies[bodyA].link_index_map[linkA] if bodyA != None and linkA != None else -1
		bulletLB = self.bodies[bodyB].link_index_map[linkB] if bodyB != None and linkB != None else -1
		contacts = []
		if bulletLA == -1 and bulletLB == -1:
			contacts = pb.getClosestPoints(bulletA, bulletB, distance=dist)
		elif bulletLA != -1 and bulletLB == -1:
			contacts = pb.getClosestPoints(bulletA, bulletB, linkIndexA=bulletLA, distance=dist)
		elif bulletLA == -1 and bulletLB != -1:
			contacts = pb.getClosestPoints(bulletA, bulletB, linkIndexB=bulletLB, distance=dist)
		else:
			contacts = pb.getClosestPoints(bulletA, bulletB, dist, bulletLA, bulletLB)
		return [ContactPoint(self.bulletIds_to_bodyIds[c[1]],                                   # Body A
							 self.bulletIds_to_bodyIds[c[2]],                                   # Body B
							 self.bodies[self.bulletIds_to_bodyIds[c[1]]].index_link_map[c[3]], # Link of A
							 self.bodies[self.bulletIds_to_bodyIds[c[2]]].index_link_map[c[4]], # Link of B
							 Vector3(*c[5]),                                                # Point on A
							 Vector3(*c[6]),                                                # Point on B
							 Vector3(*c[7]),                                                # Normal from B to A
							 c[8],                                                              # Distance
							 c[9])                                                              # Normal force
				for c in contacts]

	#def create_fixed_constraint(self, bodyA, bodyB, linkA=None, linkB=None):

	# Returns (colId, visId, type)
	def __create_vc_shapes(self, dl_shape, base_frame, color):
		if not DLRigidObject.is_a(dl_shape):
			raise Exception('Cannot create Bullet-shapes for {} because it is not even a rigid body.'.format(str(dl_shape)))

		# rel_link_tf = base_frame.inv() * dl_shape.pose
		# rel_pos = pos(rel_link_tf)
		# rel_rot =
		if DLCube.is_a(dl_shape):
			half_extents = [dl_shape.length * 0.5, dl_shape.width * 0.5, dl_shape.height * 0.5]
			colId = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=half_extents)
			#visId = pb.createVisualShape(pb.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
			return colId#, visId
		elif DLCylinder.is_a(dl_shape):
			colId = pb.createCollisionShape(pb.GEOM_CYLINDER, radius=dl_shape.radius, height=dl_shape.height*0.5)
			#visId = pb.createVisualShape(pb.GEOM_CYLINDER, radius=dl_shape.radius, height=dl_shape.height)#, rgbaColor=color)
			return colId#, visId
		elif DLSphere.is_a(dl_shape):
			colId = pb.createCollisionShape(pb.GEOM_SPHERE, radius=dl_shape.radius)
			#visId = pb.createVisualShape(pb.GEOM_SPHERE, radius=dl_shape.radius, rgbaColor=color)
			return colId#, visId
		else:
			raise Exception('Cannot create Bullet-shapes for {}'.format(str(dl_shape)))
