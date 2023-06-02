import numpy    as np
import pybullet as pb

from dataclasses import dataclass
from typing      import Union

from .frame    import Frame
from .geometry import Transform, \
                      Point3,    \
                      Vector3,   \
                      Quaternion

# Link state structure. Assigns names to bullet's info structure.
@dataclass
class LinkState:
    com_pose            : Transform
    local_inertial_pose : Transform
    world_pose          : Transform
    linear_velocity     : Vector3
    angular_velocity    : Vector3


# Link state structure. Assigns names to bullet's info structure.
@dataclass
class LinkState:
    com_pose            : Transform
    local_inertial_pose : Transform
    world_pose          : Transform
    linear_velocity     : Vector3
    angular_velocity    : Vector3


class Link(Frame):
    def __init__(self, simulator, multibody, idx : int, name : str):
        super().__init__(None)

        self._simulator = simulator
        self._client_id = simulator.client_id
        self._multibody = multibody
        self._idx       = idx
        self._name      = name

        self.__current_state = None
        self.__last_sim_pose_update = -1
        self.__current_aabb = None
        self.__last_aabb_update = -1

    @property
    def bId(self):
        return self._multibody.bId

    @property
    def name(self):
        return self._name

    @property
    def idx(self):
        return self._idx

    @property
    def local_pose(self) -> Transform:
        return self.state.world_pose
    
    @property
    def state(self):
        if self._simulator.sim_step != self.__last_sim_pose_update:
            if self._idx == -1:  # Bullet does not handle base joints the same
                world_pose = self._multibody.pose
                self.__current_state = LinkState(world_pose, 
                                                 Transform.identity(), 
                                                 world_pose,
                                                 self._multibody.linear_velocity,
                                                 self._multibody.angular_velocity)
            else:
                ls = pb.getLinkState(self._multibody.bId, 
                                     self._idx, 0, physicsClientId=self._client_id)
                self.__current_state = LinkState(Transform(Point3(*ls[0]), Quaternion(*ls[1])),
                                                 Transform(Point3(*ls[2]), Quaternion(*ls[3])),
                                                 Transform(Point3(*ls[4]), Quaternion(*ls[5])),
                                                 Vector3.zero(),
                                                 Vector3.zero())
        return self.__current_state

    @property
    def aabb(self):
        if self._simulator.sim_step != self.__last_aabb_update:
            res = pb.getAABB(self._multibody.bId, self._idx, physicsClientId=self._client_id)
            self.__current_aabb = AABB(Point3(*res[0]), Point3(*res[1]))
        return self.__current_aabb

    def reset(self):
        self.__current_aabb  = None
        self.__current_state = None
        self.__last_sim_pose_update = -1
        self.__last_aabb_update     = -1

    def apply_force(self, force : Vector3, point : Point3):
        pb.applyExternalForce(self._bulletId, \
                              self._idx, \
                              force, \
                              point, \
                              pb.WORLD_FRAME, \
                              self._client_id)

    def apply_local_force(self, force : Vector3, point : Point3):
        pb.applyExternalForce(self._bulletId, \
                              self._idx, \
                              force, \
                              point, \
                              pb.LINK_FRAME, \
                              self._client_id)

    def apply_torque(self, torque : Vector3):
        pb.applyExternalTorque(self._bulletId, \
                               self._idx, \
                               torque, \
                               Point3.zero(), \
                               pb.WORLD_FRAME, \
                               self._client_id)

    def apply_local_torque(self, torque : Vector3):
        pb.applyExternalTorque(self._bulletId, \
                               self._idx, \
                               torque, \
                               Point3.zero(), \
                               pb.LINK_FRAME, \
                               self._client_id)
    
    def jacobian(self, q, q_dot, q_ddot, point=Point3.zero()):
        j_pos, j_rot = pb.calculateJacobian(self._multibody.bId,
                                            self._idx,
                                            point,
                                            q,
                                            q_dot,
                                            q_ddot,
                                            self._client_id)
        return np.vstack((j_pos, j_rot))

    def ik(self, world_pose : Union[Point3, Transform], max_iterations=50):
        if type(world_pose) == Point3:
            return np.asarray(pb.calculateInverseKinematics(self._multibody.bId,
                                                            self._idx,
                                                            world_pose,
                                                            maxNumIterations=max_iterations,
                                                            residualThreshold=1e-5,
                                                            physicsClientId=self._client_id
                                                            ))
        return np.asarray(pb.calculateInverseKinematics(self._multibody.bId,
                                                            self._idx,
                                                            world_pose.position,
                                                            world_pose.quaternion,
                                                            maxNumIterations=max_iterations,
                                                            residualThreshold=1e-5,
                                                            physicsClientId=self._client_id
                                                            ))
    
    def get_contacts(self, other=None):
        if other is not None:
            if isinstance(other, Link):
                contacts = pb.getContactPoints(self._multibody._bulletId, other._bulletId, linkIndexA=self._idx, linkIndexB=other.idx, physicsClientId=self._client_id)
            else:
                contacts = pb.getContactPoints(self._multibody._bulletId, other._bulletId, linkIndexA=self._idx, physicsClientId=self._client_id)
        else:
            contacts = pb.getContactPoints(self._multibody._bulletId, -1, linkIndexA=self._idx, physicsClientId=self._client_id)
        return [self._simulator._decode_contact_point(cp) for cp in contacts]

    def get_closest_points(self, other=None, dist=0.2):
        if other is not None:
            if isinstance(other, Link):
                contacts = pb.getClosestPoints(self._multibody._bulletId, other._bulletId, linkIndexA=self._idx, linkIndexB=other.idx, distance=dist, physicsClientId=self._client_id)
            else:
                contacts = pb.getClosestPoints(self._multibody._bulletId, other._bulletId, linkIndexA=self._idx, distance=dist, physicsClientId=self._client_id)
        else:
            contacts = pb.getClosestPoints(self._multibody._bulletId, -1, linkIndexA=self._idx, distance=dist, physicsClientId=self._client_id)
        return [self._simulator._decode_contact_point(cp) for cp in contacts]
