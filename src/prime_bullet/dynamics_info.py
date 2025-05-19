import numpy    as np
import pybullet as pb

from dataclasses import dataclass

@dataclass
class DynamicsInfo:
    mass : float
    lateral_friction  : float
    # Diagonal of inertial matrix
    inertia           : np.ndarray
    # Position of CoM in local frame of reference
    inertia_xyz       : np.ndarray
    # Orientation of inertial frame in local frame
    inertia_xyzw      : np.ndarray
    restitution       : float
    rolling_friction  : float
    spinning_friction : float
    contact_damping   : float
    contact_stiffness : float
    body_type         : int
    collision_margin  : float

    def __post_init__(self):
        if not isinstance(self.inertia, np.ndarray):
            self.inertia = np.asarray(self.inertia)
        if not isinstance(self.inertia_xyz, np.ndarray):
            self.inertia_xyz = np.asarray(self.inertia_xyz)
        if not isinstance(self.inertia_xyzw, np.ndarray):
            self.inertia_xyzw = np.asarray(self.inertia_xyzw)


class DynamicInfoAccessor:
    def __init__(self,
                 body_id,
                 link_id,
                 client_id):
        self._body_id    = body_id
        self._link_id    = link_id
        self._client_id  = client_id
        self._info_cache = None
    
    @property
    def _info(self) -> DynamicsInfo:
        if self._info_cache is None:
            self._info_cache = DynamicsInfo(*pb.getDynamicsInfo(self._body_id,
                                                                self._link_id,
                                                                self._client_id))
        return self._info_cache

    @property
    def mass(self) -> float:
        return self._info.mass
    
    @mass.setter
    def mass(self, new_value : float):
        pb.changeDynamics(self._body_id, self._link_id, 
                          mass=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None

    @property
    def lateral_friction(self) -> float:
        return self._info.lateral_friction
    
    @lateral_friction.setter
    def lateral_friction(self, new_value : float):
        pb.changeDynamics(self._body_id, self._link_id, 
                          lateralFriction=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None

    @property
    def spinning_friction(self) -> float:
        return self._info.spinning_friction
    
    @spinning_friction.setter
    def spinning_friction(self, new_value : float):
        pb.changeDynamics(self._body_id, self._link_id, 
                          spinningFriction=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None
    
    @property
    def rolling_friction(self) -> float:
        return self._info.rolling_friction
    
    @rolling_friction.setter
    def rolling_friction(self, new_value : float):
        pb.changeDynamics(self._body_id, self._link_id, 
                          rollingFriction=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None

    @property
    def restitution(self) -> float:
        return self._info.restitution
    
    @restitution.setter
    def restitution(self, new_value : float):
        pb.changeDynamics(self._body_id, self._link_id, 
                          restitution=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None

    # GETTER IS APPARENTLY NOT EXPOSED
    # @property
    # def linear_damping(self) -> float:
    #     return self._info.linear_damping
    
    # @linear_damping.setter
    # def linear_damping(self, new_value : float):
    #     pb.changeDynamics(self._body_id, self._link_id, 
    #                       linear_damping=new_value,
    #                       physicsClientId=self._client_id)
    #     self._info_cache = None

    # @property
    # def angluar_damping(self) -> float:
    #     return self._info.angluar_damping
    
    # @angluar_damping.setter
    # def angluar_damping(self, new_value : float):
    #     pb.changeDynamics(self._body_id, self._link_id, 
    #                       angluar_damping=new_value,
    #                       physicsClientId=self._client_id)
    #     self._info_cache = None

    @property
    def contact_stiffness(self) -> float:
        return self._info.contact_stiffness
    
    @contact_stiffness.setter
    def contact_stiffness(self, new_value : float):
        pb.changeDynamics(self._body_id, self._link_id, 
                          contactStiffness=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None

    @property
    def contact_damping(self) -> float:
        return self._info.contact_damping
    
    @contact_damping.setter
    def contact_damping(self, new_value : float):
        pb.changeDynamics(self._body_id, self._link_id, 
                          contactDamping=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None

    @property
    def inertia(self) -> np.ndarray:
        return self._info.inertia
    
    @inertia.setter
    def inertia(self, new_value : np.ndarray):
        pb.changeDynamics(self._body_id, self._link_id, 
                          localInertiaDiagonal=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None

    @property
    def inertia_xyz(self) -> np.ndarray:
        return self._info.inertia_xyz
    
    @property
    def inertia_xyzw(self) -> np.ndarray:
        return self._info.inertia_xyzw

    @property
    def collision_margin(self) -> float:
        return self._info.collision_margin
    
    @collision_margin.setter
    def collision_margin(self, new_value : float):
        pb.changeDynamics(self._body_id, self._link_id, 
                          collisionMargin=new_value,
                          physicsClientId=self._client_id)
        self._info_cache = None
