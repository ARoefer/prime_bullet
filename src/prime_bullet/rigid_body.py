import pybullet as pb
import tempfile


from dataclasses import dataclass
from hashlib     import md5
from jinja2      import Template
from omegaconf   import OmegaConf
from pathlib     import Path
from typing      import Union

from . import IAI_BULLET_ROOT
from .frame    import Frame
from .geometry import Point3,     \
                      Quaternion, \
                      Vector3,    \
                      Transform,  \
                      AABB
from .utils    import ColorRGBA,  \
                      res_pkg_path
from .link import Link

# Mapping of bullet's geometry constants to internal keywords
GEOM_SPHERE   = 'sphere'
GEOM_BOX      = 'box'
GEOM_CYLINDER = 'cylinder'
GEOM_CAPSULE  = 'capsule'
GEOM_MESH     = 'mesh'


class RigidBody(Frame):
    @dataclass
    class Config:
        initial_pose : Transform
        current_pose : Transform
        type         : str


    """Wrapper class giving object oriented access to PyBullet's rigid bodies.
    """
    def __init__(self, simulator, 
                       bulletId,
                       type,
                       initial_pose=Transform.identity()):
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
        super(RigidBody, self).__init__(None)

        self._simulator     = simulator
        self._client_id     = simulator.client_id
        self._bulletId      = bulletId

        self._initial_pose  = initial_pose
        
        self.type           = type

        self.__current_pose = None
        self.__last_sim_pose_update = -1
        self.__current_lin_velocity = None
        self.__current_ang_velocity = None
        self.__last_sim_velocity_update = -1
        self._conf_type = RigidBody.Config

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
        self._simulator.register_deletion_cb(self._simulator.get_body_id(self.bId), cb)

    def deregister_deletion_cb(self, cb):
        """Deregisters a callback function which is called when this object is deleted.

        :param cb: Callback to be called. Signature f(BasicSimulator, str, RigidBody/MultiBody)
        :tyoe  cb: function
        """
        self._simulator.deregister_deletion_cb(self._simulator.get_body_id(self.bId), cb)

    def reset(self):
        """Resets this object's pose and joints to their initial configuration."""
        pb.resetBasePositionAndOrientation(self._bulletId, 
                                           self._initial_pose.position, 
                                           self._initial_pose.quaternion, 
                                           physicsClientId=self._client_id)
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
    def local_pose(self) -> Transform:
        if self._simulator.sim_step != self.__last_sim_pose_update:
            temp = pb.getBasePositionAndOrientation(self._bulletId, physicsClientId=self._client_id)
            self.__current_pose = Transform(Point3(*temp[0]), Quaternion(*temp[1]))
            self.__last_sim_pose_update = self._simulator.sim_step

        return self.__current_pose

    @local_pose.setter
    def local_pose(self, pose):
        pb.resetBasePositionAndOrientation(self._bulletId,
                                           pose.position,
                                           pose.quaternion,
                                           physicsClientId=self._client_id)
        self.__last_sim_pose_update = -1
    
    @property
    def pose(self):
        return super().pose

    @pose.setter
    def pose(self, pose):
        self.local_pose = pose

    @property
    def linear_velocity(self):
        """Returns the object's current linear velocity.
        :rtype: list
        """
        if self._simulator.sim_step != self.__last_sim_velocity_update:
            temp = pb.getBaseVelocity(self._bulletId, physicsClientId=self._client_id)
            self.__current_lin_velocity, self.__current_ang_velocity = Vector3(*temp[0]), Vector3(*temp[1])
            self.__last_sim_velocity_update = self._simulator.sim_step
        return self.__current_lin_velocity

    @property
    def angular_velocity(self):
        """Returns the object's current angular velocity.
        :rtype: list
        """
        if self._simulator.sim_step != self.__last_sim_velocity_update:
            temp = pb.getBaseVelocity(self._bulletId, physicsClientId=self._client_id)
            self.__current_lin_velocity, self.__current_ang_velocity = Vector3(temp[0]), Vector3(temp[1])
            self.__last_sim_velocity_update = self._simulator.sim_step
        return self.__current_ang_velocity

    @property
    def initial_pose(self):
        return self._initial_pose

    @initial_pose.setter
    def initial_pose(self, pose):
        self._initial_pose = pose

    def apply_force(self, force : Vector3, point : Point3):
        pb.applyExternalForce(self._bulletId, \
                              -1, \
                              force, \
                              point, \
                              pb.WORLD_FRAME, \
                              self._client_id)

    def apply_local_force(self, force : Vector3, point : Point3):
        pb.applyExternalForce(self._bulletId, \
                              -1, \
                              force, \
                              point, \
                              pb.LINK_FRAME, \
                              self._client_id)

    def apply_torque(self, torque : Vector3):
        pb.applyExternalTorque(self._bulletId, \
                               -1, \
                               torque, \
                               Point3.zero(), \
                               pb.WORLD_FRAME, \
                               self._client_id)

    def apply_local_torque(self, torque : Vector3):
        pb.applyExternalTorque(self._bulletId, \
                               -1, \
                               torque, \
                               Point3.zero(), \
                               pb.LINK_FRAME, \
                               self._client_id)

    def conf(self) -> OmegaConf:
        out = self._conf_type()
        out.current_pose = self.pose
        out.initial_pose = self._initial_pose
        out.type = self.type
        return out

    def get_contacts(self, other=None):
        if other is not None:
            if isinstance(other, RigidBody):
                contacts = pb.getContactPoints(self._bulletId, other._bulletId, physicsClientId=self._client_id)
            elif isinstance(other, Link):
                contacts = pb.getContactPoints(self._bulletId, other._bulletId, linkIndexB=other.idx, physicsClientId=self._client_id)
            else:
                raise Exception(f'Unsupported type for contact checking "{type(other)}". Need to inherit from RigidBody or Link.')
        else:
            contacts = pb.getContactPoints(self._bulletId, -1, physicsClientId=self._client_id)
        return [self._simulator._decode_contact_point(cp) for cp in contacts]

    def get_closest_points(self, other=None, dist=0.2):
        if other is not None:
            if isinstance(other, RigidBody):
                contacts = pb.getClosestPoints(self._bulletId, other._bulletId, distance=dist, physicsClientId=self._client_id)
            elif isinstance(other, Link):
                contacts = pb.getClosestPoints(self._bulletId, other._bulletId, linkIndexB=other.idx, distance=dist, physicsClientId=self._client_id)
            else:
                raise Exception(f'Unsupported type for contact checking "{type(other)}". Need to inherit from RigidBody or Link.')
        else:
            contacts = pb.getClosestPoints(self._bulletId, -1, distance=dist, physicsClientId=self._client_id)
        return [self._simulator._decode_contact_point(cp) for cp in contacts]


with open(f'{IAI_BULLET_ROOT}/data/urdf/template_box.urdf', 'r') as f:
    _TEMPLATE_BOX = Template(f.read())

with open(f'{IAI_BULLET_ROOT}/data/urdf/template_cylinder.urdf', 'r') as f:
    _TEMPLATE_CYLINDER = Template(f.read())

with open(f'{IAI_BULLET_ROOT}/data/urdf/template_sphere.urdf', 'r') as f:
    _TEMPLATE_SPHERE = Template(f.read())

with open(f'{IAI_BULLET_ROOT}/data/urdf/template_single_mesh.urdf', 'r') as f:
    _TEMPLATE_MESH = Template(f.read())


class BoxBody(RigidBody):
    @dataclass
    class Config(RigidBody.Config):
        size : Vector3
        mass : float
        color: Vector3

    def __init__(self, simulator, 
                       size=Vector3(1, 1, 1),
                       initial_pose=Transform.identity(), 
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
                               initial_pose.position,
                               initial_pose.quaternion,
                               0,
                               0,
                               pb.URDF_MERGE_FIXED_LINKS,
                               physicsClientId=simulator.client_id)
        
        super().__init__(simulator, bulletId, 'box', initial_pose)
        self._conf_type = BoxBody.Config

    def conf(self) -> OmegaConf:
        out = super().conf()
        out.size  = self.size
        out.mass  = self.mass
        out.color = self.color
        return out


class CylinderBody(RigidBody):
    @dataclass
    class Config(RigidBody.Config):
        radius : float
        length : float
        mass   : float
        color  : Vector3

    def __init__(self, simulator, 
                       radius=0.5,
                       length=1.0,
                       initial_pose=Transform.identity(), 
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
                               initial_pose.position,
                               initial_pose.quaternion,
                               0,
                               0,
                               pb.URDF_MERGE_FIXED_LINKS,
                               physicsClientId=simulator.client_id)
        
        super().__init__(simulator, bulletId, 'cylinder', initial_pose)

    def conf(self) -> OmegaConf:
        out = super().conf()
        out.radius = self.radius
        out.length = self.length
        out.mass   = self.mass
        out.color  = self.color
        return out


class SphereBody(RigidBody):
    @dataclass
    class Config(RigidBody.Config):
        radius : float
        mass   : float
        color  : Vector3

    def __init__(self, simulator, 
                       radius=0.5,
                       initial_pose=Transform.identity(), 
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
                               initial_pose.position,
                               initial_pose.quaternion,
                               0,
                               0,
                               pb.URDF_MERGE_FIXED_LINKS,
                               physicsClientId=simulator.client_id)
        
        super().__init__(simulator, bulletId, 'sphere', initial_pose)

    def conf(self) -> OmegaConf:
        out = super().conf()
        out.radius = self.radius
        out.mass   = self.mass
        out.color  = self.color
        return out


class MeshBody(RigidBody):
    @dataclass
    class Config(RigidBody.Config):
        scale : float
        visual_mesh : str
        collision_mesh : str
        mass  : float
        color : Vector3

    def __init__(self, simulator, 
                       visual_mesh,
                       collision_mesh=None,
                       scale=1,
                       initial_pose=Transform.identity(), 
                       color=ColorRGBA(0, 0, 1, 1),
                       mass=1):
        self.scale = scale
        self.mass  = mass
        self.color = color
        self.visual_mesh = visual_mesh
        self.collision_mesh = collision_mesh

        file_hash = md5(f'mesh_{collision_mesh}_{visual_mesh}_{scale}_{mass}_{color}'.encode('utf-8')).hexdigest()
        fpath = Path(f'{tempfile.gettempdir()}/{file_hash}.urdf')
        if not fpath.exists():
            with open(str(fpath), 'w') as f:
                f.write(_TEMPLATE_MESH.render(mass=mass,
                                              mesh_path=res_pkg_path(visual_mesh), 
                                              collision_path=res_pkg_path(collision_mesh) if collision_mesh is not None else res_pkg_path(visual_mesh), 
                                              scale=scale, 
                                              color=color))
        
        try:
            bulletId = pb.loadURDF(str(fpath),
                                   initial_pose.position,
                                   initial_pose.quaternion,
                                   0,
                                   0,
                                   pb.URDF_MERGE_FIXED_LINKS,
                                   physicsClientId=simulator.client_id)
        except pb.error as e:
            raise Exception(f'Exception raised during load of generated urdf "{fpath}": {e}')
        
        super().__init__(simulator, bulletId, 'mesh', initial_pose)

    def conf(self) -> OmegaConf:
        out = super().conf()
        out.scale = self.scale
        out.visual_mesh = self.visual_mesh
        out.collision_mesh = self.collision_mesh
        out.mass  = self.mass
        out.color = self.color
        return out


class SDFBody(RigidBody):
    @dataclass
    class Config(RigidBody.Config):
        sdf_path : str

    def __init__(self, simulator, bulletId, sdf_path):
        self.sdf_path = sdf_path

        super().__init__(simulator, bulletId, 'sdf_mesh', None)
        self.initial_pose = self.pose

    def conf(self) -> OmegaConf:
        out = super().conf()
        out.sdf_path = self.sdf_path
        return out


class SDFWorldBody(RigidBody):
    def __init__(self, simulator, bulletId):        
        super().__init__(simulator, bulletId, 'mesh', None)
        self.initial_pose = self.pose

    def conf(self) -> OmegaConf:
        return None

