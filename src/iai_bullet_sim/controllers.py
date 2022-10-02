import numpy as np

from typing import Union

from iai_bullet_sim.geometry  import Vector3,  \
                                     Point3,   \
                                     Transform
from iai_bullet_sim.multibody import MultiBody, Link


class CartesianRelativeController(object):
    def __init__(self, robot : MultiBody, link : Link):
        self._robot = robot
        self._link  = link
        self.reset()

    def act(self, delta : Union[np.ndarray, Transform]):
        self._pose = self._link.pose.dot(delta)

        ik_solution = self._link.ik(self._pose)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)

    def reset(self):
        self._pose = self._link.pose

    @property
    def goal(self):
        return self._pose


class CartesianRelativePointController(object):
    def __init__(self, robot : MultiBody, link : Link):
        self._robot = robot
        self._link  = link
        self.reset()

    def act(self, delta : Union[np.ndarray, Vector3]):
        self._point = self._link.pose.position + delta

        ik_solution = self._link.ik(self._point)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)
    
    def reset(self):
        self._point = self._link.pose.position

    @property
    def goal(self):
        return self._point


class CartesianRelativeVirtualPointController(object):
    def __init__(self, robot : MultiBody, link : Link):
        self._robot = robot
        self._link  = link
        self.reset()
    
    def act(self, delta : Union[np.ndarray, Vector3]):
        self._point += delta

        ik_solution = self._link.ik(self._point)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)
    
    def reset(self):
        self._point = self._link.pose.position

    @property
    def goal(self):
        return self._point
