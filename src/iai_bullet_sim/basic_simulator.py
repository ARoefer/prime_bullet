import pybullet as pb
import random
from collections import namedtuple
from iai_bullet_sim.utils import res_pkg_path, rot3_to_quat, Vector3, Quaternion, Frame
from pprint import pprint


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


class JointDriver(object):
	def update_positions(self, robot_data, positions_dict):
		pass
	def update_velocities(self, robot_data, velocities_dict):
		pass
	def update_effort(self, robot_data, effort_dict):
		pass

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
		pb.resetBasePositionAndOrientation(self.bulletId, self.initial_pos, self.initial_rot)
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

	def get_AABB(self, bodyId, linkId=None):
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
		if override_initial:
			self.initial_pos = pos
			self.initial_rot = quat


	def set_joint_positions(self, state, override_initial=False):
		for j, p in state.items():
			pb.resetJointState(self.__bulletId, self.joints[j].jointIndex, p)
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


ReactionForces  = namedtuple('ReactionForces', ['f', 'm'])

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType', 'qIndex', 'uIndex',
									 'flags', 'jointDamping', 'jointFriction', 'lowerLimit',
									 'upperLimit', 'maxEffort', 'maxVelocity', 'linkName',
									 'axis', 'parentFramePos', 'parentFrameOrn', 'parentIndex'])

class JointState(object):
	def __init__(self, pos, vel, rForce, appliedTorque):
		self.position = pos
		self.velocity = vel
		self.reactionForce = ReactionForces(rForce[:3], rForce[3:])
		self.effort = appliedTorque

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
	temp = pb.invertTransform(list(frame_tuple.position), list(frame_tuple.quaternion))
	return Frame(Vector3(*temp[0]), Quaternion(*temp[1]))


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


class BasicSimulator(object):
	def __init__(self, tick_rate):
		self.bodies    = {}
		self.constraints = {}
		self.tick_rate = tick_rate
		self.time_step = 1.0 / self.tick_rate
		self.__n_updates = 0
		self.__bId_IdMap = {}

		self.__h = random.random()
		self.__nextId = 0
		self.__joint_types = {'FIXED': pb.JOINT_FIXED, 'PRISMATIC': pb.JOINT_PRISMATIC, 'HINGE': pb.JOINT_POINT2POINT}

		self.__plugins = set()

	def get_n_update(self):
		return self.__n_updates

	def __gen_next_color(self):
		self.__h += 0.618033988749895
		self.__h %= 1.0
		return hsva_to_rgba(self.__h, 0.7, 0.95, 1.0)

	def init(self, gravity=[0,0,-9.81], mode='direct'):
		self.physicsClient = pb.connect({'gui': pb.GUI, 'direct': pb.DIRECT}[mode])#or p.DIRECT for non-graphical version
		pb.setGravity(*gravity)
		pb.setTimeStep(self.time_step)

	def kill(self):
		pb.disconnect()

	def update(self):
		for plugin in self.__plugins:
			plugin.pre_physics_update(self, self.time_step)
		
		pb.stepSimulation()
		self.__n_updates += 1
		
		for plugin in self.__plugins:
			plugin.post_physics_update(self, self.time_step)


	def reset(self):
		for body in self.bodies.values():
			self.reset(body)

	def register_object(self, obj):
		base_link, bodyName = pb.getBodyInfo(obj.bId())
		bodyId =  bodyName
		counter = 0
		while bodyId in self.bodies:
			bodyId = '{}.{}'.format(bodyName, counter)
			counter += 1
		self.bodies[bodyId] = obj
		self.__bId_IdMap[obj.bId()] = bodyId
		return bodyId

	def register_plugin(self, plugin):
		self.__plugins.add(plugin)

	def deregister_plugin(self, plugin):
		self.__plugins.remove(plugin)

	def load_urdf(self, urdf_path, pos=[0,0,0], rot=[0,0,0,1], joint_driver=JointDriver(), useFixedBase=0, name_override=None):
		res_urdf_path = res_pkg_path(urdf_path)
		print('Simulator: {}'.format(res_urdf_path))

		new_body = MultiBody(self, pb.loadURDF(res_urdf_path,
							 	               pos,
							                   rot,
							                   0,              # MAXIMAL COORDINATES, DO NOT TOUCH!
							                   useFixedBase,
							                   flags=pb.URDF_USE_SELF_COLLISION), self.__gen_next_color(), pos, rot, joint_driver, urdf_path)
		

		bodyId = self.register_object(new_body)
		print('created new robot with id {}'.format(bodyId))
		return new_body

	def get_body_id(self, bulletId):
		if bulletId in self.__bId_IdMap:
			return self.__bId_IdMap[bulletId]
		return None


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

	def load_world(self, world_dict):
		if 'objects' in world_dict:
			if type(world_dict['objects']) != list:
				raise Exception('Field "objects" in world dictionary needs to be of type list.')

			for od in world_dict['objects']:
				type  = od['type']
				name  = od['name']
				i_pos = od['initial_pose']['position']
				i_rot = od['initial_pose']['orientation']

				if type == 'multibody':
					urdf_path = od['urdf_path']
					fixed_base = od['fixed_base']
					initial_joint_state = od['joint_position']
					new_obj = self.load_urdf(urdf_path, i_pos, i_rot, joint_driver=JointDriver(), useFixedBase=fixed_base, name_override=name)
					new_obj.set_joint_positions(initial_joint_state, True)
					for s in od['sensors']:
						new_obj.enable_joint_sensor(s, True)
				else:
					raise Exception('Unknown object type "{}"'.format(type))

	def save_world(self, use_current_state_as_init=False):
		out = {'objects': [], 'constraints': []}

		for bname, b in self.bodies.items():
			if type(b) == MultiBody:
				od = {'name': bname, 
					  'type': 'multibody', 
					  'initial_pose': {
					  	'position': b.initial_pos,
					  	'rotation': b.initial_rot}, 
				  	  'urdf_path': b.urdf_file,
				  	  'initial_joint_state': b.initial_joint_state,
				  	  'fixed_base': True} # TODO: Update this!
		  		out['objects'].append(od)
			else:
				raise Exception('Can not serialize type "{}"'.format(str(type(b))))


		for cname, c in self.constraints.items():
			pass

		return out		


class SimulatorPlugin(object):
	"""docstring for SimulatorPlugin"""
	def __init__(self, name):
		super(SimulatorPlugin, self).__init__()
		self.__name = name

	def pre_physics_update(self, simulator, deltaT):
		pass

	def post_physics_update(self, simulator, deltaT):
		pass

	def __str__(self):
		return self.__name
		