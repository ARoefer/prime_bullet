import numpy as np

from typing import Union

from iai_bullet_sim.geometry  import Vector3,  \
                                     Point3,   \
                                     Transform
from iai_bullet_sim.multibody import MultiBody, Link


class JointPositionController(object):
    def __init__(self, robot : MultiBody):
        self._robot = robot
        self._q_goal = None
        self.reset()

    def act(self, position : Union[np.ndarray, Transform]):
        self._q_goal = position
        self._robot.apply_joint_pos_cmds(self._q_goal, 
                                         self._robot.q_f_max)

    def reset(self):
        self._q_goal = self._robot.q

    @property
    def delta(self):
        return self._q_goal - self._robot.q

    @property
    def goal(self):
        return self._q_goal

class TestController(object):
    def __init__(self, robot : MultiBody, link : Link):
        self._robot = robot
        self._link  = link
        self.reset()

    def set_target(self, target_pose : Transform, thresh_lin : float = 0.05, thresh_ang : float = np.deg2rad(15)):
        current_pose = self._link.pose
        tf_delta = current_pose.relative(target_pose)
        lin_dist = tf_delta.position.norm()
        ang_dist = tf_delta.quaternion.angle()

        thresh_lin = 0.05
        thresh_ang = np.deg2rad(15)

        lin_steps = int(lin_dist / thresh_lin)
        ang_steps = int(ang_dist / thresh_ang)

        n_steps = max(1, lin_steps, ang_steps)

        increment_steps = [(1 / n_steps) * s for s in range(1, n_steps + 1)]

        points = [current_pose.lerp(target_pose, step) for step in increment_steps]

        return points


    def act(self, position : Union[np.ndarray, Transform], force_scale : float = 1.0):
        self._transform = position

        ik_solution = self._link.ik(self._transform)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max*force_scale)

    def reset(self):
        self._transform = self._link.pose

    @property
    def delta(self):
        ee_pose = self._link.pose
        return np.array([(self._transform.position - ee_pose.position).norm(), 
                          self._transform.quaternion.angle(ee_pose.quaternion)])

    @property
    def goal(self):
        return self._transform

class CartesianController(object):
    def __init__(self, robot : MultiBody, link : Link):
        self._robot = robot
        self._link  = link
        self.reset()

    def act(self, goal : Transform):
        self._transform = goal

        ik_solution = self._link.ik(self._transform)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)

    def reset(self):
        self._transform = self._link.pose

    @property
    def delta(self):
        ee_pose = self._link.pose
        return np.array([(self._transform.position - ee_pose.position).norm(), 
                          self._transform.quaternion.angle(ee_pose.quaternion)])

    @property
    def goal(self):
        return self._transform


class CartesianRelativeController(object):
    def __init__(self, robot : MultiBody, link : Link):
        self._robot = robot
        self._link  = link
        self.reset()

    def act(self, delta : Union[np.ndarray, Transform]):
        self._transform = self._link.pose.dot(delta)

        ik_solution = self._link.ik(self._transform)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)

    def reset(self):
        self._transform = self._link.pose

    @property
    def delta(self):
        ee_pose = self._link.pose
        return np.array([(self._transform.position - ee_pose.position).norm(), 
                          self._transform.quaternion.angle(ee_pose.quaternion)])

    @property
    def goal(self):
        return self._transform


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
    def delta(self):
        return (self._point - self._link.pose.position).norm()

    @property
    def goal(self):
        return self._point


class CartesianRelativePointCOrientationController(object):
    def __init__(self, robot : MultiBody, link : Link):
        self._robot = robot
        self._link  = link
        self.reset()

    def act(self, delta : Union[np.ndarray, Vector3]):
        self._transform.position = self._link.pose.position + delta

        ik_solution = self._link.ik(self._transform)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)
    
    def reset(self):
        self._transform = self._link.pose

    @property
    def delta(self):
        ee_pose = self._link.pose
        return np.array([(self._transform.position - ee_pose.position).norm(), 
                          self._transform.quaternion.angle(ee_pose.quaternion)])

    @property
    def goal(self):
        return self._transform


class CartesianRelativeVirtualPointController(object):
    def __init__(self, robot : MultiBody, link : Link, max_delta=1.0):
        self._robot = robot
        self._link  = link
        self._max_vp_delta = max_delta
        self.reset()
    
    def act(self, delta : Union[np.ndarray, Vector3]):
        self._point += delta

        ee_vp_delta = self._transform.position - self._link.pose.position

        # Clip goal point back to max distance from link
        if ee_vp_delta.norm() > self._max_vp_delta:
            self._transform.position = self._link.pose.position + ee_vp_delta.normalized() * self._max_vp_delta

        ik_solution = self._link.ik(self._point)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)
    
    def reset(self):
        self._point = self._link.pose.position

    @property
    def delta(self):
        return (self._position - self._link.pose.position).norm()
    
    @property
    def goal(self):
        return self._point


class CartesianRelativeVPointCOrientationController(object):
    def __init__(self, robot : MultiBody, link : Link, max_delta=1.0):
        self._robot = robot
        self._link  = link
        self._max_vp_delta = max_delta
        self.reset()
    
    def act(self, delta : Union[np.ndarray, Vector3]):
        self._transform.position += delta

        ee_vp_delta = self._transform.position - self._link.pose.position

        # Clip goal point back to max distance from link
        if ee_vp_delta.norm() > self._max_vp_delta:
            self._transform.position = self._link.pose.position + ee_vp_delta.normalized() * self._max_vp_delta

        ik_solution = self._link.ik(self._transform)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)
    
    def reset(self):
        self._transform = self._link.pose

    @property
    def delta(self):
        ee_pose = self._link.pose
        return np.array([(self._transform.position - ee_pose.position).norm(), 
                          self._transform.quaternion.angle(ee_pose.quaternion)])

    @property
    def goal(self):
        return self._transform

# Virtual point variable orientation controller
class CartesianRelativeVPointVOrientationController(object):
    def __init__(self, robot : MultiBody, link : Link, max_delta=1.0):
        self._robot = robot
        self._link  = link
        self._max_vp_delta = max_delta
        self.reset()
    
    def act(self, delta : Union[np.ndarray, Transform]):
        self._transform.position += delta.position
        self._transform.quaternion = self._transform.quaternion.dot(delta.quaternion)

        ee_vp_delta = self._transform.position - self._link.pose.position

        # Clip goal point back to max distance from link
        if ee_vp_delta.norm() > self._max_vp_delta:
            self._transform.position = self._link.pose.position + ee_vp_delta.normalized() * self._max_vp_delta

        ik_solution = self._link.ik(self._transform)
        self._robot.apply_joint_pos_cmds(ik_solution, 
                                         self._robot.q_f_max)
    
    def reset(self):
        self._transform = self._link.pose

    @property
    def delta(self):
        ee_pose = self._link.pose
        return np.array([(self._transform.position - ee_pose.position).norm(), 
                          self._transform.quaternion.angle(ee_pose.quaternion)])

    @property
    def goal(self):
        return self._transform