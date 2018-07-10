import pybullet as pb
from collections import namedtuple
from iai_bullet_sim.utils import Vector3, Quaternion, Frame

BULLET_GEOM_TYPES = {pb.GEOM_SPHERE: 'sphere', pb.GEOM_BOX: 'box', pb.GEOM_CYLINDER: 'cylinder', pb.GEOM_CAPSULE: 'capsule'}
GEOM_TYPES = {v: k for k, v in BULLET_GEOM_TYPES.items()}


class RigidBody(object):
	def __init__(self, simulator, bulletId, geom_type, color, initial_pos=[0,0,0], initial_rot=[0,0,0,1], halfExtents=[0.5,0.5,0.5], radius=0.5, height=1, mass=1):
		if geom_type not in GEOM_TYPES:
			raise Exception('Rigid body type needs to be {}'.format(' or '.join(['"{}"'.format(t) for t in GEOM_TYPES])))

		self.simulator      = simulator
		self.__bulletId     = bulletId
		self.type           = geom_type
		self.color          = color

		self.initial_pos    = initial_pos
		self.initial_rot    = initial_rot

		self.halfExtents    = halfExtents
		self.radius         = radius
		self.height         = height
		self.mass           = mass

		self.__current_pose = None
		self.__last_sim_pose_update = -1

	def bId(self):
		return self.__bulletId

	def reset(self):
		pb.resetBasePositionAndOrientation(self.__bulletId, self.initial_pos, self.initial_rot)
		self.__last_sim_pose_update = -1

	def get_AABB(self, linkId=None):
		res = pb.getAABB(self.__bulletId, -1)
		return AABB(Vector3(*res[0]), Vector3(*res[1]))

	def pose(self):
		if self.simulator.get_n_update() != self.__last_sim_pose_update:
			temp = pb.getBasePositionAndOrientation(self.__bulletId)
			self.__current_pose = Frame(temp[0], temp[1])
		return self.__current_pose

	def set_pose(self, pose, override_initial=False):
		pos  = pose.position
		quat = pose.quaternion
		pb.resetBasePositionAndOrientation(self.__bulletId, pos, quat)
		self.__last_sim_pose_update = -1
		if override_initial:
			self.initial_pos = list(pos)
			self.initial_rot = list(quat)


	def get_contacts(self, other_body=None, other_link=None):
		return self.simulator.get_contacts(self, other_body, None, other_link)

	def get_closest_points(self, other_body=None, other_link=None):
		return self.simulator.get_closest_points(self, other_body, None, other_link)