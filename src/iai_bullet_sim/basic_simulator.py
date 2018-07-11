import pybullet as pb
import random
from collections import namedtuple
from iai_bullet_sim.utils import res_pkg_path, rot3_to_quat, Vector3, Quaternion, Frame
from iai_bullet_sim.multibody import MultiBody, JointDriver
from iai_bullet_sim.rigid_body import RigidBody, GEOM_TYPES, BULLET_GEOM_TYPES
from pprint import pprint

# Constraint structure. Assigns names to bullet's info structure.
Constraint = namedtuple('Constraint', ['bulletId', 'bodyParent', 'bodyChild', 'linkParent', 'linkChild',
						  			   'jointType', 'jointAxis', 'parentJointPosition', 'childJointPosition',
						  			   'parentJointOrientation', 'childJointOrientation'])

# Visual shape structure. Assigns names to bullet's info structure.
VisualShape = namedtuple('VisualShape', ['bulletId', 'linkIndex', 'geometryType', 'dimensions', 'filename', 'localPosition', 'localOrientation', 'rgba'])

# Collision shape structure. Assigns names to bullet's info structure.
CollisionShape = namedtuple('CollisionShape', ['bulletId', 'linkIndex', 'geometryType', 'dimensions', 'filename', 'localPosition', 'localOrientation'])

# Axis aligned bounding box structure. Represents AABBs as a tuple of a low and high corner.
AABB = namedtuple('AABB', ['min', 'max'])


def hsva_to_rgba(h, s, v, a):
	"""Converts a HSVA color to a RGBA color."""
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
	"""Converts an indexable structure with len >= 3 to a list containing the first three elements."""
	return [vec[0], vec[1], vec[2]]

def vec_add(a, b):
	"""Performs per element addition on two indexable structures with len >= 3 and returns the result as Vector3."""
	return Vector3(a[0] + b[0], a[1] + b[1], a[2] + b[2])

def vec_sub(a, b):
	"""Performs per element subtraction on two indexable structures with len >= 3 and returns the result as Vector3."""
	return Vector3(a[0] - b[0], a[1] - b[1], a[2] - b[2])

def vec_scale(a, x):
	"""Performs per element multiplication on an indexable structure with len >= 3 and returns the result as Vector3."""
	return Vector3(a.x * x, a.y * x, a.z * x)

def invert_transform(frame_tuple):
	"""Inverts the transformation represented by the Frame datatype and returns it as new frame."""
	temp = pb.invertTransform(list(frame_tuple.position), list(frame_tuple.quaternion))
	return Frame(Vector3(*temp[0]), Quaternion(*temp[1]))


class ContactPoint(object):
	"""Wrapper for bullet's contact point structure."""
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
		"""Compares the distances of two contact points."""
		return self.dist <= other.dist


class BasicSimulator(object):
	"""Class wrapping the PyBullet interface in an object oriented manner."""
	def __init__(self, tick_rate=50, gravity=[0,0,-9.81]):
		"""Constructs a simulator.

		tick_rate: Ticks ideally performed per second.
		gravity: Gravity force for the simulation.
		"""
		self.physicsClient = None
		self.bodies      = {}
		self.constraints = {}
		self.tick_rate   = tick_rate
		self.gravity     = gravity
		self.time_step   = 1.0 / self.tick_rate
		self.__n_updates = 0
		self.__bId_IdMap = {}

		self.__h = random.random()
		self.__nextId = 0
		self.__joint_types = {'FIXED': pb.JOINT_FIXED, 'PRISMATIC': pb.JOINT_PRISMATIC, 'HINGE': pb.JOINT_POINT2POINT}

		self.__plugins = set()

	def get_n_update(self):
		"""Returns the number of performed updates."""
		return self.__n_updates

	def __gen_next_color(self):
		"""Internal. Generates a new random color."""
		self.__h += 0.618033988749895
		self.__h %= 1.0
		return hsva_to_rgba(self.__h, 0.7, 0.95, 1.0)


	def init(self, mode='direct'):
		"""Initializes the connection to Bullet.

		mode: Mode of the connection. Options: gui | direct
		"""
		self.physicsClient = pb.connect({'gui': pb.GUI, 'direct': pb.DIRECT}[mode])#or p.DIRECT for non-graphical version
		pb.setGravity(*self.gravity)
		pb.setTimeStep(self.time_step)


	def set_tick_rate(self, tick_rate):
		"""Updates the tick rate of the simulation."""
		self.tick_rate = tick_rate
		self.time_step = 1.0 / self.tick_rate
		if self.physicsClient is not None:
			pb.setTimeStep(self.time_step)			


	def set_gravity(self, gravity):
		"""Updates the simulations gravity."""
		self.gravity = gravity
		if self.physicsClient is not None:
			pb.setGravity(*gravity)

	def stop(self):
		"""Stops the simulation. Calls disable() on all plugins."""
		for plugin in self.__plugins:
			plugin.disable(self)

	def kill(self):
		"""Kills the connection to Bullet."""
		pb.disconnect()

	def pre_update(self):
		"""Called before every physics step."""
		for plugin in self.__plugins:
			plugin.pre_physics_update(self, self.time_step)
		
	def physics_update(self):
		"""Steps the physics simulation."""
		pb.stepSimulation()
		self.__n_updates += 1
		
	def post_update(self):
		"""Called after every physics step."""
		for plugin in self.__plugins:
			plugin.post_physics_update(self, self.time_step)

	def update(self):
		"""Performs one complete update, consisting of pre-, physics- and post update."""
		self.pre_update()
		self.physics_update()
		self.post_update()		
		
		
	def reset(self):
		"""Resets all bodies in the simulation to their initial state."""
		for body in self.bodies.values():
			self.reset(body)

	def register_object(self, obj, name_override=None):
		"""Registers an object with the simulator.
		Unless a specific name is given, the simulator will automatically assign one to the object.
		If the specific name is already taken an exception will be raised.

		obj: Object to register with the simulator.
		name_override: Name to assign to the object.
		"""
		if name_override is None:
			if isinstance(obj, MultiBody):
				base_link, bodyId = pb.getBodyInfo(obj.bId())
			elif isinstance(obj, RigidBody):
				bodyId = obj.type
			counter = 0
			bodyName = bodyId
			while bodyId in self.bodies:
				bodyId = '{}.{}'.format(bodyName, counter)
				counter += 1

			self.bodies[bodyId] = obj
			self.__bId_IdMap[obj.bId()] = bodyId
			return bodyId
		else:
			if name_override in self.bodies:
				raise Exception('Id "{}" is already taken.'.format(name_override))
			
			self.bodies[name_override] = obj
			self.__bId_IdMap[obj.bId()] = name_override
			return name_override


	def register_plugin(self, plugin):
		"""Registers a plugin with the simulator."""
		self.__plugins.add(plugin)

	def deregister_plugin(self, plugin):
		"""Removes a plugin from the simulator's registry."""
		self.__plugins.remove(plugin)

	def load_urdf(self, urdf_path, pos=[0,0,0], rot=[0,0,0,1], joint_driver=JointDriver(), useFixedBase=0, name_override=None):
		"""Loads an Object from a URDF and registers it with this simulator.

		urdf_path: Path of the file as local or global path, or as ROS package URI.
		pos: Position to create the object at.
		rot: Rotation to create the object with.
		joint_driver: Custom joint driver for custom joint behavior.
		useFixedBase: Should the base of the object be fixed in space?
		name_override: Custom name to assign to this object during registration.
		"""
		res_urdf_path = res_pkg_path(urdf_path)
		print('Simulator: {}'.format(res_urdf_path))

		new_body = MultiBody(self, pb.loadURDF(res_urdf_path,
							 	               pos,
							                   rot,
							                   0,              # MAXIMAL COORDINATES, DO NOT TOUCH!
							                   useFixedBase,
							                   flags=pb.URDF_USE_SELF_COLLISION), self.__gen_next_color(), pos, rot, joint_driver, urdf_path)
		

		bodyId = self.register_object(new_body, name_override)
		print('Created new multibody with id {}'.format(bodyId))
		return new_body


	def create_sphere(self, radius=0.5, pos=[0,0,0], rot=[0,0,0,1], mass=1, color=None, name_override=None):
		return self.create_object(BULLET_GEOM_TYPES[pb.GEOM_SPHERE], radius=radius, pos=pos, rot=rot, mass=mass, color=color, name_override=name_override)

	def create_box(self, half_extents=[0.5]*3, pos=[0,0,0], rot=[0,0,0,1], mass=1, color=None, name_override=None):
		return self.create_object(BULLET_GEOM_TYPES[pb.GEOM_BOX], half_extents=half_extents, pos=pos, rot=rot, mass=mass, color=color, name_override=name_override)

	def create_cylinder(self, radius=0.5, height=1, pos=[0,0,0], rot=[0,0,0,1], mass=1, color=None, name_override=None):
		return self.create_object(BULLET_GEOM_TYPES[pb.GEOM_CYLINDER], radius=radius, height=height, pos=pos, rot=rot, mass=mass, color=color, name_override=name_override)

	def create_capsule(self, radius=0.5, height=1, pos=[0,0,0], rot=[0,0,0,1], mass=1, color=None, name_override=None):
		return self.create_object(BULLET_GEOM_TYPES[pb.GEOM_CAPSULE], radius=radius, height=height, pos=pos, rot=rot, mass=mass, color=color, name_override=name_override)


	def create_object(self, geom_type, half_extents=[0.5,0.5,0.5], radius=0.5, height=1, pos=[0,0,0], rot=[0,0,0,1], mass=1, color=None, name_override=None):

		if geom_type not in GEOM_TYPES:
			raise Exception('Unknown geometry type "{}". Options are: {}'.format(geom_type, ', '.join(geom_type.keys())))

		if color is None:
			color = self.__gen_next_color()

		new_body = RigidBody(self, 
							 pb.createRigidBody(GEOM_TYPES[geom_type], radius, half_extents, height, mass, pos, rot, color),
							 geom_type, color, pos, rot, half_extents, radius, height, mass)
		bodyId = self.register_object(new_body, name_override)
		print('Created new rigid body with id {}'.format(bodyId))
		return new_body


	def get_body_id(self, bulletId):
		if bulletId in self.__bId_IdMap:
			return self.__bId_IdMap[bulletId]
		return None

	def get_body(self, bodyId):
		if bodyId in self.bodies:
			return self.bodies[bodyId]
		return None


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
	def get_overlapping(self, aabb, filter=set()):
		raw_overlap = pb.getOverlappingObjects(vec3_to_list(aabb.min), vec3_to_list(aabb.max))
		if raw_overlap == None:
			return []

		return [self.__get_obj_link_tuple(bulletId, linkIdx) for bulletId, linkIdx in raw_overlap if self.bodies[self.__bId_IdMap[bulletId]] not in filter]

	# @profile
	def get_contacts(self, bodyA=None, bodyB=None, linkA=None, linkB=None):
		bulletA = bodyA.bId() if bodyA != None else -1
		bulletB = bodyB.bId() if bodyB != None else -1
		bulletLA = bodyA.link_index_map[linkA] if bodyA != None and linkA != None and isinstance(bodyA, MultiBody) else -1
		bulletLB = bodyB.link_index_map[linkB] if bodyB != None and linkB != None and isinstance(bodyB, MultiBody) else -1
		contacts = []
		if bulletLA == -1 and bulletLB == -1:
			contacts = pb.getContactPoints(bulletA, bulletB)
		elif bulletLA != -1 and bulletLB == -1:
			contacts = pb.getContactPoints(bulletA, bulletB, linkIndexA=bulletLA)
		elif bulletLA == -1 and bulletLB != -1:
			contacts = pb.getContactPoints(bulletA, bulletB, linkIndexB=bulletLB)
		else:
			contacts = pb.getContactPoints(bulletA, bulletB, bulletLA, bulletLB)
		return [self.__create_contact_point(c) for c in contacts]


	# @profile
	def get_closest_points(self, bodyA, bodyB, linkA=None, linkB=None, dist=0.2):
		bulletA = bodyA.bId() if bodyA != None else -1
		bulletB = bodyB.bId() if bodyB != None else -1
		bulletLA = bodyA.link_index_map[linkA] if bodyA != None and linkA != None and isinstance(bodyA, MultiBody) else -1
		bulletLB = bodyB.link_index_map[linkB] if bodyB != None and linkB != None and isinstance(bodyB, MultiBody) else -1
		contacts = []
		if bulletLA == -1 and bulletLB == -1:
			contacts = pb.getClosestPoints(bulletA, bulletB, distance=dist)
		elif bulletLA != -1 and bulletLB == -1:
			contacts = pb.getClosestPoints(bulletA, bulletB, linkIndexA=bulletLA, distance=dist)
		elif bulletLA == -1 and bulletLB != -1:
			contacts = pb.getClosestPoints(bulletA, bulletB, linkIndexB=bulletLB, distance=dist)
		else:
			contacts = pb.getClosestPoints(bulletA, bulletB, dist, bulletLA, bulletLB)
		return [self.__create_contact_point(c) for c in contacts]


	def load_world(self, world_dict):
		if 'objects' in world_dict:
			if type(world_dict['objects']) != list:
				raise Exception('Field "objects" in world dictionary needs to be of type list.')

			for od in world_dict['objects']:
				otype  = od['type']
				name  = od['name']
				i_pos = od['initial_pose']['position']
				i_rot = od['initial_pose']['rotation']

				if otype == 'multibody':
					urdf_path = od['urdf_path']
					fixed_base = od['fixed_base']
					initial_joint_state = od['initial_joint_state']
					new_obj = self.load_urdf(urdf_path, i_pos, i_rot, joint_driver=JointDriver(), useFixedBase=fixed_base, name_override=name)
					new_obj.set_joint_positions(initial_joint_state, True)
					for s in od['sensors']:
						new_obj.enable_joint_sensor(s, True)
				elif otype == 'rigid_body':
					self.create_object(od['geom_type'], od['extents'], od['radius'], od['height'], i_pos, i_rot, od['mass'], od['color'], name)
				else:
					raise Exception('Unknown object type "{}"'.format(otype))
		if 'constraints' in world_dict:
			pass


	def save_world(self, use_current_state_as_init=False):
		out = {'objects': [], 'constraints': []}

		for bname, b in self.bodies.items():
			if isinstance(b, MultiBody):
				in_pos = b.pose().position if use_current_state_as_init else b.initial_pos
				in_rot = b.pose().quaternion if use_current_state_as_init else b.initial_rot
				if use_current_state_as_init:
					in_js = {j: p.position for j, p in b.joint_state().items()}
				else:
					in_js = b.initial_joint_state

				od = {'name': bname, 
					  'type': 'multibody', 
					  'initial_pose': {
					  	'position': list(in_pos),
					  	'rotation': list(in_rot)}, 
				  	  'urdf_path': b.urdf_file,
				  	  'initial_joint_state': in_js,
				  	  'fixed_base': True,
				  	  'sensors': list(b.joint_sensors)} # TODO: Update this!
		  		out['objects'].append(od)
	  		elif isinstance(b, RigidBody):
	  			in_pos = b.pose().position if use_current_state_as_init else b.initial_pos
				in_rot = b.pose().quaternion if use_current_state_as_init else b.initial_rot

	  			od = {'name': bname, 
					  'type': 'rigid_body',
					  'geom_type': b.type,
					  'initial_pose': {
					  	'position': list(in_pos),
					  	'rotation': list(in_rot)}, 
				  	  'color': list(b.color),
				  	  'mass': b.mass,
				  	  'extents': list(b.halfExtents),
				  	  'radius': b.radius,
				  	  'height': b.height} # TODO: Update this!
		  		out['objects'].append(od)
			else:
				raise Exception('Can not serialize type "{}"'.format(str(type(b))))


		for cname, c in self.constraints.items():
			pass

		return out		

	def load_simulator(self, config_dict, plugin_registry):
		if 'tick_rate' in config_dict:
			self.set_tick_rate(config_dict['tick_rate'])
		
		if 'gravity' in config_dict:
			self.set_gravity(config_dict['gravity'])
		
		if 'world' in config_dict:
			self.load_world(config_dict['world'])

		if 'plugins' in config_dict:
			for plugin_dict in config_dict['plugins']:
				if plugin_dict['plugin_type'] not in plugin_registry:
					print('Unknown plugin type: {}'.format(plugin_dict['plugin_type']))
					continue

				self.register_plugin(plugin_registry[plugin_dict['plugin_type']].factory(self, plugin_dict))

	def save_simulator(self, use_current_state_as_init=False):
		out = {'tick_rate': self.tick_rate,
			   'gravity': self.gravity,
			   'world': self.save_world(use_current_state_as_init), 
			   'plugins': []}

		for plugin in self.__plugins:
			pdict = {'plugin_type': str(type(plugin))}
			pdict.update(plugin.to_dict(self))
			out['plugins'].append(pdict)
		return out


	def __create_contact_point(self, bcp):
		bodyA, linkA = self.__get_obj_link_tuple(c[1], c[3])
		bodyB, linkB = self.__get_obj_link_tuple(c[2], c[4])
		return ContactPoint(bodyA,          # Body A
							bodyB,          # Body B
							linkA, 			# Link of A
							linkB, 			# Link of B
							Vector3(*c[5]), # Point on A
							Vector3(*c[6]), # Point on B
							Vector3(*c[7]), # Normal from B to A
							c[8],           # Distance
							c[9])           # Normal force

	def __get_obj_link_tuple(self, bulletId, linkIdx):
		body = self.bodies[self.__bId_IdMap[bulletId]]
		link = body.index_link_map[linkIdx] if isinstance(body, MultiBody) else None
		return (body, link)


class SimulatorPlugin(object):
	"""docstring for SimulatorPlugin"""
	def __init__(self, name):
		super(SimulatorPlugin, self).__init__()
		self.__name = name

	def pre_physics_update(self, simulator, deltaT):
		pass

	def post_physics_update(self, simulator, deltaT):
		pass

	def disable(self, simulator):
		pass

	def __str__(self):
		return self.__name

	def to_dict(self, simulator):
		raise (NotImplementedError)
		