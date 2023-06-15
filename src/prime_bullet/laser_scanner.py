import numpy as np
import pybullet as pb

from iai_bullet_sim.frame           import Frame
from iai_bullet_sim.geometry        import Transform
from iai_bullet_sim.rigid_body      import RigidBody
from iai_bullet_sim.multibody       import Link


class LaserScanner(Frame):
    def __init__(self, simulator,
                       min_rad,
                       max_rad,
                       steps,
                       range_min,
                       range_max,
                       pose, # Will be ignored if parent is RigidBody or link
                       parent=None) -> None:
        super().__init__(parent)

        self._simulator = simulator
        self._pose = pose if not isinstance(parent, RigidBody) and not isinstance(parent, Link) else Transform.identity()
        self._min_rad = min_rad
        self._max_rad = max_rad
        self._steps   = steps
        self._range_min = range_min
        self._range_max = range_max
        self.__range_scale = range_max - range_min

        angles = np.linspace(min_rad, max_rad, steps)
        self._rays_start = np.vstack((np.cos(angles) * range_min, 
                                      np.sin(angles) * range_min,
                                      np.zeros(len(angles)),
                                      np.ones(len(angles))))
        self._rays_end   = np.vstack((np.cos(angles) * range_max, 
                                      np.sin(angles) * range_max,
                                      np.zeros(len(angles)),
                                      np.ones(len(angles))))
        
        if isinstance(parent, RigidBody) and isinstance(parent, Link):
            self._rays_start = self._rays_start[:3].T
            self._rays_end   = self._rays_end[:3].T
            self._parent_object_id = self._parent.bId
            self._parent_link_id = -1 if not isinstance(self._parent, Link) else self._parent.idx
        else:
            self._parent_object_id = None
            self._parent_link_id = None

    @property
    def local_pose(self) -> Transform:
        return self._pose

    @local_pose.setter
    def local_pose(self, pose) -> None:
        self._local_pose = pose

    @property
    def pose(self) -> Transform:
        return super().pose
    
    @pose.setter
    def pose(self, world_pose) -> None:
        w_to_p = Transform.identity()
        if self._parent is not None:
            w_to_p = self._parent.pose.inv()
        self.local_pose = w_to_p.dot(world_pose)

    def render(self):
        if self._parent_object_id is None:
            s_to_w = self.pose.matrix()
            rays_start = s_to_w.dot(self._rays_start)[:3].T
            rays_end   = s_to_w.dot(self._rays_end)[:3].T
            results = pb.rayTestBatch(rays_start, rays_end, self._simulator.client_id)
        else:
            results = pb.rayTestBatch(self._rays_start, 
                                      self._rays_end,
                                      self._parent_object_id,
                                      self._parent_link_id,
                                      physicsClientId=self._simulator.client_id)
        ids, links, frac, pos, normal = zip(*results) 
        return np.asarray(frac) * self.__range_scale + self._range_min
