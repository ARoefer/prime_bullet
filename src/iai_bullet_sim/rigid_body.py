import pybullet as pb
from collections import namedtuple
from iai_bullet_sim.utils import Vector3, Quaternion, Frame

# Mapping of bullet's geometry constants to internal keywords
BULLET_GEOM_TYPES = {pb.GEOM_SPHERE: 'sphere', pb.GEOM_BOX: 'box', pb.GEOM_CYLINDER: 'cylinder', pb.GEOM_CAPSULE: 'capsule'}
# Mapping of internal keywords to bullet's geometry constants
GEOM_TYPES = {v: k for k, v in BULLET_GEOM_TYPES.items()}


class RigidBody(object):
    """Wrapper class giving object oriented access to PyBullet's rigid bodies.
    """
    def __init__(self, simulator, bulletId, geom_type, color, initial_pos=[0,0,0], initial_rot=[0,0,0,1], halfExtents=[0.5,0.5,0.5], radius=0.5, height=1, mass=1):
        """Constructs a rigid body.
        
        simulator   -- The simulator managing this object
        bulletId    -- The Id of the corresponding bullet object
        geom_type   -- Shape of this object. sphere | box | cylinder | capsule 
        color       -- A color override for this object
        initial_pos -- This object's initial location
        initial_rot -- This object's initial rotation
        halfExtents -- Half edge lengths for box type.
        radius      -- Radius for sphere, cylinder and capsule
        height      -- Total height of cylinder and capsule.
        mass        -- Mass of the object. 0 = static 
        """
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
        """Returns the corresponding bullet Id"""
        return self.__bulletId

    def reset(self):
        """Resets this object's pose and joints to their initial configuration."""
        pb.resetBasePositionAndOrientation(self.__bulletId, self.initial_pos, self.initial_rot)
        self.__last_sim_pose_update = -1

    def get_AABB(self):
        """Returns the bounding box of this object."""
        res = pb.getAABB(self.__bulletId, -1)
        return AABB(Vector3(*res[0]), Vector3(*res[1]))

    def pose(self):
        """Returns the object's current pose in the form of a Frame."""
        if self.simulator.get_n_update() != self.__last_sim_pose_update:
            temp = pb.getBasePositionAndOrientation(self.__bulletId)
            self.__current_pose = Frame(temp[0], temp[1])
        return self.__current_pose

    def set_pose(self, pose, override_initial=False):
        """Sets the current pose of the object.

        override_initial -- Additionally set the given pose as initial pose.
        """
        pos  = pose.position
        quat = pose.quaternion
        pb.resetBasePositionAndOrientation(self.__bulletId, pos, quat)
        self.__last_sim_pose_update = -1
        if override_initial:
            self.initial_pos = list(pos)
            self.initial_rot = list(quat)


    def get_contacts(self, other_body=None, other_link=None):
        """Gets the contacts this body had during the last physics step.
        The contacts can be filtered by other bodies and their links.
        
        other_body -- Other body to filter by
        other_link -- Other object's link to filter by.
        """
        return self.simulator.get_contacts(self, other_body, None, other_link)

    def get_closest_points(self, other_body=None, other_link=None):
        """Gets the closest points of this body to its environment.
        The closest points can be filtered by other bodies and their links.
        
        other_body -- Other body to filter by
        other_link -- Other object's link to filter by
        """
        return self.simulator.get_closest_points(self, other_body, None, other_link)