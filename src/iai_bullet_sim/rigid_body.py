import pybullet as pb
from iai_bullet_sim.utils import Vector3, Frame, AABB

# Mapping of bullet's geometry constants to internal keywords
BULLET_GEOM_TYPES = {pb.GEOM_SPHERE: 'sphere', pb.GEOM_BOX: 'box', pb.GEOM_CYLINDER: 'cylinder', pb.GEOM_CAPSULE: 'capsule'}
# Mapping of internal keywords to bullet's geometry constants
GEOM_TYPES = {v: k for k, v in BULLET_GEOM_TYPES.items()}


class RigidBody(object):
    """Wrapper class giving object oriented access to PyBullet's rigid bodies.
    """
    def __init__(self, simulator, bulletId, geom_type, color, initial_pos=[0,0,0], initial_rot=[0,0,0,1], extents=[1,1,1], radius=0.5, height=1, mass=1):
        """Constructs a rigid body.

        :param simulator:   The simulator managing this object
        :type  simulator:   iai_bullet_sim.basic_simulator.BasicSimulator
        :param bulletId:    The Id of the corresponding bullet object
        :type  bulletId:    long
        :param geom_type:   Shape of this object. sphere | box | cylinder | capsule
        :type  geom_type:   str
        :param color:       A color override for this object as RGBA
        :type  color:       list
        :param initial_pos: This object's initial location
        :type  initial_pos: list
        :param initial_rot: This object's initial rotation
        :type  initial_rot: list
        :param extents:     Edge lengths for box type.
        :type  extents:     list
        :param radius:      Radius for sphere, cylinder and capsule
        :type  radius:      float
        :param height:      Total height of cylinder and capsule.
        :type  height:      float
        :param mass:        Mass of the object. 0 = static
        :type  mass:        float
        """
        if geom_type not in GEOM_TYPES:
            raise Exception('Rigid body type needs to be {}'.format(' or '.join(['"{}"'.format(t) for t in GEOM_TYPES])))

        self.simulator      = simulator
        self.__client_id    = simulator.client_id()
        self.__bulletId     = bulletId
        self.type           = geom_type
        self.color          = color

        self.initial_pos    = initial_pos
        self.initial_rot    = initial_rot

        self.extents        = extents
        self.radius         = radius
        self.height         = height
        self.mass           = mass

        self.__current_pose = None
        self.__last_sim_pose_update = -1
        self.__current_lin_velocity = None
        self.__current_ang_velocity = None
        self.__last_sim_velocity_update = -1

    def bId(self):
        """Returns the corresponding bullet Id.
        :rtype: long
        """
        return self.__bulletId

    def register_deletion_cb(self, cb):
        """Registers a callback function which is called when this object is deleted.

        :param cb: Callback to be called. Signature f(BasicSimulator, str, RigidBody/MultiBody)
        :tyoe  cb: function
        """
        self.simulator.register_deletion_cb(self.simulator.get_body_id(self.bId()), cb)

    def deregister_deletion_cb(self, cb):
        """Deregisters a callback function which is called when this object is deleted.

        :param cb: Callback to be called. Signature f(BasicSimulator, str, RigidBody/MultiBody)
        :tyoe  cb: function
        """
        self.simulator.deregister_deletion_cb(self.simulator.get_body_id(self.bId()), cb)

    def reset(self):
        """Resets this object's pose and joints to their initial configuration."""
        pb.resetBasePositionAndOrientation(self.__bulletId, self.initial_pos, self.initial_rot, physicsClientId=self.__client_id)
        self.__last_sim_pose_update = -1
        self.__last_sim_velocity_update = -1

    def get_AABB(self):
        """Returns the bounding box of this object.
        :rtype: AABB
        """
        res = pb.getAABB(self.__bulletId, -1, physicsClientId=self.__client_id)
        return AABB(Vector3(*res[0]), Vector3(*res[1]))

    def pose(self):
        """Returns the object's current pose in the form of a Frame.
        :rtype: Frame
        """
        if self.simulator.get_n_update() != self.__last_sim_pose_update:
            temp = pb.getBasePositionAndOrientation(self.__bulletId, physicsClientId=self.__client_id)
            self.__current_pose = Frame(temp[0], temp[1])
            self.__last_sim_pose_update = self.simulator.get_n_update()

        return self.__current_pose

    def linear_velocity(self):
        """Returns the object's current linear velocity.
        :rtype: list
        """
        if self.simulator.get_n_update() != self.__last_sim_velocity_update:
            self.__current_lin_velocity, self.__current_ang_velocity = pb.getBaseVelocity(self.__bulletId, physicsClientId=self.__client_id)
            self.__last_sim_velocity_update = self.simulator.get_n_update()
        return self.__current_lin_velocity

    def angular_velocity(self):
        """Returns the object's current angular velocity.
        :rtype: list
        """
        if self.simulator.get_n_update() != self.__last_sim_velocity_update:
            self.__current_lin_velocity, self.__current_ang_velocity = pb.getBaseVelocity(self.__bulletId, physicsClientId=self.__client_id)
            self.__last_sim_velocity_update = self.simulator.get_n_update()
        return self.__current_ang_velocity

    def set_pose(self, pose, override_initial=False):
        """Sets the current pose of the object.

        :param override_initial: Additionally set the given pose as initial pose.
        :type  override_initial: bool
        """
        pos  = pose.position
        quat = pose.quaternion
        pb.resetBasePositionAndOrientation(self.__bulletId, pos, quat, physicsClientId=self.__client_id)
        self.__last_sim_pose_update = -1
        if override_initial:
            self.initial_pos = list(pos)
            self.initial_rot = list(quat)


    def get_contacts(self, other_body=None, other_link=None):
        """Gets the contacts this body had during the last physics step.
        The contacts can be filtered by other bodies and their links.

        :param other_body: Other body to filter by
        :type  other_body: iai_bullet_sim.multibody.MultiBody, RigidBody, NoneType
        :param other_link: Other object's link to filter by.
        :type  other_link: str, NoneType
        :rtype: list
        """
        return self.simulator.get_contacts(self, other_body, None, other_link)

    def get_closest_points(self, other_body=None, other_link=None, dist=0.2):
        """Gets the closest points of this body to its environment.
        The closest points can be filtered by other bodies and their links.

        :param other_body: Other body to filter by
        :type  other_body: iai_bullet_sim.multibody.MultiBody, RigidBody, NoneType
        :param other_link: Other object's link to filter by.
        :type  other_link: str, NoneType
        :param dist:       Maximum distance to search. Greater distance -> more expensive
        :type  dist:       float
        :rtype: list
        """
        return self.simulator.get_closest_points(self, other_body, None, other_link, dist)