from pathlib import Path
import pybullet as pb
import tempfile


from hashlib import md5
from jinja2  import Template

from iai_bullet_sim import IAI_BULLET_ROOT
from iai_bullet_sim.utils import ColorRGBA,  \
                                 Point3,     \
                                 Quaternion, \
                                 Vector3,    \
                                 Transform,  \
                                 AABB

# Mapping of bullet's geometry constants to internal keywords
GEOM_SPHERE   = 'sphere'
GEOM_BOX      = 'box'
GEOM_CYLINDER = 'cylinder'
GEOM_CAPSULE  = 'capsule'
GEOM_MESH     = 'mesh'


class RigidBody(object):
    """Wrapper class giving object oriented access to PyBullet's rigid bodies.
    """
    def __init__(self, simulator, 
                       bulletId,
                       type,
                       initial_pos=Point3(0,0,0), 
                       initial_rot=Quaternion(0,0,0,1)):
        """Constructs)a rigid body.

        :param simulator:   The simulator managing this object
        :type  simulator:   iai_bullet_sim.basic_simulator.BasicSimulator
        :param bulletId:    The Id of the corresponding bullet object
        :type  bulletId:    long
        :param initial_pos: This object's initial location
        :type  initial_pos: list
        :param initial_rot: This object's initial rotation
        :type  initial_rot: list
        :param mass:        Mass of the object. 0 = static
        :type  mass:        float
        """
        self._simulator     = simulator
        self._client_id     = simulator.client_id
        self._bulletId      = bulletId

        self._initial_pos   = initial_pos
        self._initial_rot   = initial_rot
        
        self.type           = type

        self.__current_pose = None
        self.__last_sim_pose_update = -1
        self.__current_lin_velocity = None
        self.__current_ang_velocity = None
        self.__last_sim_velocity_update = -1

    @property
    def bId(self):
        """Returns the corresponding bullet Id.
        :rtype: long
        """
        return self._bulletId

    def register_deletion_cb(self, cb):
        """Registers a callback function which is called when this object is deleted.

        :param cb: Callback to be called. Signature f(BasicSimulator, str, RigidBody/MultiBody)
        :tyoe  cb: function
        """
        self.simulator.register_deletion_cb(self.simulator.get_body_id(self.bId), cb)

    def deregister_deletion_cb(self, cb):
        """Deregisters a callback function which is called when this object is deleted.

        :param cb: Callback to be called. Signature f(BasicSimulator, str, RigidBody/MultiBody)
        :tyoe  cb: function
        """
        self.simulator.deregister_deletion_cb(self.simulator.get_body_id(self.bId), cb)

    def reset(self):
        """Resets this object's pose and joints to their initial configuration."""
        pb.resetBasePositionAndOrientation(self._bulletId, self.initial_pos, self.initial_rot, physicsClientId=self._client_id)
        self.__last_sim_pose_update = -1
        self.__last_sim_velocity_update = -1

    @property
    def aabb(self):
        """Returns the bounding box of this object.
        :rtype: AABB
        """
        res = pb.getAABB(self._bulletId, -1, physicsClientId=self._client_id)
        return AABB(Point3(*res[0]), Point3(*res[1]))

    @property
    def pose(self):
        """Returns the object's current pose in the form of a Frame.
        :rtype: Frame
        """
        if self._simulator.sim_step != self.__last_sim_pose_update:
            temp = pb.getBasePositionAndOrientation(self._bulletId, physicsClientId=self._client_id)
            self.__current_pose = Transform(Point3(*temp[0]), Quaternion(*temp[1]))
            self.__last_sim_pose_update = self._simulator.sim_step

        return self.__current_pose

    @pose.setter
    def pose(self, pose):
        pb.resetBasePositionAndOrientation(self._bulletId,
                                           pose.position,
                                           pose.quaternion,
                                           physicsClientId=self._client_id)
        self.__last_sim_pose_update = -1

    @property
    def linear_velocity(self):
        """Returns the object's current linear velocity.
        :rtype: list
        """
        if self.simulator.sim_step != self.__last_sim_velocity_update:
            temp = pb.getBaseVelocity(self._bulletId, physicsClientId=self._client_id)
            self.__current_lin_velocity, self.__current_ang_velocity = Vector3(temp[0]), Vector3(temp[1])
            self.__last_sim_velocity_update = self.simulator.sim_step
        return self.__current_lin_velocity

    @property
    def angular_velocity(self):
        """Returns the object's current angular velocity.
        :rtype: list
        """
        if self.simulator.sim_step != self.__last_sim_velocity_update:
            temp = pb.getBaseVelocity(self._bulletId, physicsClientId=self._client_id)
            self.__current_lin_velocity, self.__current_ang_velocity = Vector3(temp[0]), Vector3(temp[1])
            self.__last_sim_velocity_update = self.simulator.sim_step
        return self.__current_ang_velocity

    @property
    def initial_pose(self):
        return Transform(self.initial_pos, self.initial_rot)

    @initial_pose.setter
    def initial_pose(self, pose):
        self.initial_pos = pose.position
        self.initial_rot = pose.quaternion

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


with open(f'{IAI_BULLET_ROOT}/data/urdf/template_box.urdf', 'r') as f:
    _TEMPLATE_BOX = Template(f.read())

with open(f'{IAI_BULLET_ROOT}/data/urdf/template_cylinder.urdf', 'r') as f:
    _TEMPLATE_CYLINDER = Template(f.read())

with open(f'{IAI_BULLET_ROOT}/data/urdf/template_sphere.urdf', 'r') as f:
    _TEMPLATE_SPHERE = Template(f.read())

with open(f'{IAI_BULLET_ROOT}/data/urdf/template_single_mesh.urdf', 'r') as f:
    _TEMPLATE_MESH = Template(f.read())


class BoxBody(RigidBody):
    def __init__(self, simulator, 
                       size=[1, 1, 1],
                       initial_pos=Point3(0, 0, 0), 
                       initial_rot=Quaternion(0, 0, 0, 1),
                       color=ColorRGBA(1, 0, 0, 1),
                       mass=1):
        self.size  = size
        self.mass  = mass
        self.color = color
        
        file_hash = md5(f'box_{size}_{mass}_{color}'.encode('utf-8')).hexdigest()
        fpath = Path(f'{tempfile.gettempdir()}/{file_hash}.urdf')
        if not fpath.exists():
            with open(str(fpath), 'w') as f:
                f.write(_TEMPLATE_BOX.render(mass=mass, size=size, color=color))
        
        bulletId = pb.loadURDF(str(fpath),
                               initial_pos,
                               initial_rot,
                               0,
                               0,
                               pb.URDF_MERGE_FIXED_LINKS | pb.URDF_ENABLE_SLEEPING,
                               physicsClientId=simulator.client_id)
        
        super().__init__(simulator, bulletId, 'box', initial_pos, initial_rot)

class CylinderBody(RigidBody):
    def __init__(self, simulator, 
                       radius=0.5,
                       length=1.0,
                       initial_pos=Point3(0, 0, 0), 
                       initial_rot=Quaternion(0, 0, 0, 1),
                       color=ColorRGBA(0, 1, 0, 1),
                       mass=1):
        self.radius = radius
        self.length = length
        self.mass   = mass
        self.color  = color
        
        file_hash = md5(f'cylinder_{radius}_{length}_{mass}_{color}'.encode('utf-8')).hexdigest()
        fpath = Path(f'{tempfile.gettempdir()}/{file_hash}.urdf')
        if not fpath.exists():
            with open(str(fpath), 'w') as f:
                f.write(_TEMPLATE_CYLINDER.render(mass=mass, radius=radius, length=length, color=color))
        
        bulletId = pb.loadURDF(str(fpath),
                               initial_pos,
                               initial_rot,
                               0,
                               0,
                               pb.URDF_MERGE_FIXED_LINKS | pb.URDF_ENABLE_SLEEPING,
                               physicsClientId=simulator.client_id)
        
        super().__init__(simulator, bulletId, 'cylinder', initial_pos, initial_rot)

class SphereBody(RigidBody):
    def __init__(self, simulator, 
                       radius=0.5,
                       initial_pos=Point3(0, 0, 0), 
                       initial_rot=Quaternion(0, 0, 0, 1),
                       color=ColorRGBA(0, 0, 1, 1),
                       mass=1):
        self.radius = radius
        self.mass   = mass
        self.color  = color
        
        file_hash = md5(f'sphere_{radius}_{mass}_{color}'.encode('utf-8')).hexdigest()
        fpath = Path(f'{tempfile.gettempdir()}/{file_hash}.urdf')
        if not fpath.exists():
            with open(str(fpath), 'w') as f:
                f.write(_TEMPLATE_SPHERE.render(mass=mass, radius=radius, color=color))
        
        bulletId = pb.loadURDF(str(fpath),
                               initial_pos,
                               initial_rot,
                               0,
                               0,
                               pb.URDF_MERGE_FIXED_LINKS | pb.URDF_ENABLE_SLEEPING,
                               physicsClientId=simulator.client_id)
        
        super().__init__(simulator, bulletId, 'sphere', initial_pos, initial_rot)

class MeshBody(RigidBody):
    def __init__(self, simulator, 
                       visual_mesh,
                       collision_mesh=None,
                       scale=1,
                       initial_pos=Point3(0, 0, 0), 
                       initial_rot=Quaternion(0, 0, 0, 1),
                       color=ColorRGBA(0, 0, 1, 1),
                       mass=1):
        self.scale = scale
        self.mass  = mass
        self.color = color
        
        file_hash = md5(f'mesh_{collision_mesh}_{visual_mesh}_{scale}_{mass}_{color}'.encode('utf-8')).hexdigest()
        fpath = Path(f'{tempfile.gettempdir()}/{file_hash}.urdf')
        if not fpath.exists():
            with open(str(fpath), 'w') as f:
                f.write(_TEMPLATE_MESH.render(mass=mass,
                                              mesh_path=visual_mesh, 
                                              collision_path=collision_mesh, 
                                              scale=scale, 
                                              color=color))
        
        bulletId = pb.loadURDF(str(fpath),
                               initial_pos,
                               initial_rot,
                               0,
                               0,
                               pb.URDF_MERGE_FIXED_LINKS | pb.URDF_ENABLE_SLEEPING,
                               physicsClientId=simulator.client_id)
        
        super().__init__(simulator, bulletId, 'mesh', initial_pos, initial_rot)