import pybullet as pb
import numpy    as np

from collections import namedtuple
from dataclasses import dataclass
from functools   import lru_cache
from typing      import Union

from iai_bullet_sim.frame      import Frame
from iai_bullet_sim.rigid_body import RigidBody
from iai_bullet_sim.geometry   import Point3, \
                                      Transform, \
                                      Vector3, \
                                      Quaternion, \
                                      AABB
from .link import Link

from math import atan2, cos, sin

class JointDriver(object):
    """Joint drivers modify given velocity and effort commands to match robot specific joint behavior.
    """
    def update_positions(self, robot_data, positions_dict):
        """Updates a given position command."""
        pass
    def update_velocities(self, robot_data, velocities_dict):
        """Updates a given velocity command."""
        pass
    def update_effort(self, robot_data, effort_dict):
        """Updates a given effort command."""
        pass

    def to_dict(self):
        return {}

    @classmethod
    def factory(cls, config_dict):
        return JointDriver()


class JointState(object):
    """Represents the state of an individual bullet joint."""
    def __init__(self, pos, vel, rForce, appliedTorque):
        """
        :type pos: float
        :type vel: float
        :type rForce: float
        :type appliedTorque: float
        """
        self.position = pos
        self.velocity = vel
        self.reactionForce = ReactionForces(Vector3(*rForce[:3]), Vector3(*rForce[3:]))
        self.effort = appliedTorque


@dataclass
class ReactionForces:
    linear  : Vector3
    angular : Vector3

    def numpy(self):
        return np.hstack([self.linear, self.angular])

# Joint info structure. Assigns names to bullet's info structure.
JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType', 'qIndex', 'uIndex',
                                     'flags', 'jointDamping', 'jointFriction', 'lowerLimit',
                                     'upperLimit', 'maxEffort', 'maxVelocity', 'linkName',
                                     'axis', 'parentFramePos', 'parentFrameOrn', 'parentIndex'])


class Joint(Frame):
    def __init__(self, body, jointIndex, jointName, jointType, qIndex, uIndex,
                             flags, jointDamping, jointFriction, lowerLimit,
                             upperLimit, maxEffort, maxVelocity, linkName,
                             axis, parentFramePos, parentFrameOrn, parentIndex):
        self.body  = body
        self.index = jointIndex
        self.name  = jointName.decode('utf-8')
        self.type  = jointType
        self.q_idx = qIndex
        self.u_idx = uIndex
        self.damping  = jointDamping
        self.friction = jointFriction
        self.q_min    = lowerLimit
        self.q_max    = upperLimit
        self.limits   = np.array([lowerLimit, upperLimit])
        self.f_max    = maxEffort
        self.qd_max   = maxVelocity
        self.link     = body.links[linkName.decode('utf-8')]
        self.axis     = Vector3(*axis)
        self._local_pose = Transform(Point3(*parentFramePos), Quaternion(*parentFrameOrn))
        self._parent     = body.get_link_by_index(parentIndex)

    @property
    def is_dynamic(self):
        return self.type != pb.JOINT_FIXED

    @property
    def local_pose(self):
        return self._local_pose

    @property
    def parent(self):
        return self._parent


class MultiBody(RigidBody):
    """Wrapper class giving object oriented access to PyBullet's multibodies.
    """
    def __init__(self, simulator, 
                       bulletId,
                       initial_pose=Transform.identity(),
                       joint_driver=JointDriver(), 
                       urdf_file=None):
        """Constructs a multibody.

        :param simulator:    The simulator managing this object
        :type  simulator:    iai_bullet_sim.basic_simulator.BasicSimulator
        :param bulletId:     The Id of the corresponding bullet object
        :type  bulletId:     long
        :param color:        A color override for this object as RGBA
        :type  color:        list
        :param initial_pos:  This object's initial location
        :type  initial_pos:  list
        :param initial_rot:  This object's initial rotation
        :type  initial_rot:  list
        :param joint_driver: A joint driver instance, which might modify the object's joints
        :type  joint_driver: JointDriver
        :param urdf_file:    The URDF this object was loaded from. This may be a path in the ROS package syntax
        :type  urdf_file:    str, NoneType
        """
        super().__init__(simulator, bulletId, 'multibody', initial_pose)

        self.joints         = {}
        self.dynamic_joints = {}
        self.joint_idx_name_map = {}
        self.joint_sensors  = {}
        self.joint_driver   = joint_driver
        self.urdf_file      = urdf_file

        self._initial_joint_state = {}

        base_link, multibodyName = pb.getBodyInfo(self._bulletId, physicsClientId=self._client_id)
        #print('PyBullet says:\n  Name: {}\n  Base: {}\n'.format(multibodyName, base_link))
        self.base_link = base_link.decode('utf-8')

        joint_info = [JointInfo(*pb.getJointInfo(self._bulletId, x, physicsClientId=self._client_id)) for x in range(pb.getNumJoints(self._bulletId, physicsClientId=self._client_id))]

        self.i_links     = {info.jointIndex: info.linkName.decode('utf-8') for info in joint_info}
        self.i_links[-1] = self.base_link
        self.i_links     = {i: Link(self._simulator, self, i, n) for i, n in self.i_links.items()}
        self.links       = {link.name: link for link in self.i_links.values()}

        self.i_joints = [Joint(self, *info) for info in joint_info]
        self.joints   = {j.name: j for j in self.i_joints}
        self.dynamic_joints       = {n: j for n, j in self.joints.items() if j.is_dynamic}
        self._initial_joint_state = {n: min(max(j.q_min, 0.0), j.q_max) for n, j in self.dynamic_joints.items()}

        self.__joint_names           = [j.name for j in self.i_joints]
        self.__dynamic_joint_indices = [info.index for info in self.dynamic_joints.values()]
        #print('dynamic joints:\n  {}'.format('\n  '.join([info.jointName for info in self.joints.values() if info.jointType != pb.JOINT_FIXED])))
        self.__monitored_joint_indices = self.__dynamic_joint_indices.copy()

        self._q        = None
        self._q_dot    = None
        self._q_f      = None
        self.q_min     = np.array([j.q_min  for j in self.i_joints if j.is_dynamic])
        self.q_max     = np.array([j.q_max  for j in self.i_joints if j.is_dynamic])
        self.q_dot_max = np.array([j.qd_max for j in self.i_joints if j.is_dynamic])
        self.q_f_max   = np.array([j.f_max  for j in self.i_joints if j.is_dynamic])

        # Initialize empty JS for objects without dynamic joints
        self._joint_state = None if len(self.__dynamic_joint_indices) > 0 else {}
        self.__last_sim_js_update = -1
        self._torque_control = False

    @property
    def torque_control(self):
        return self._torque_control

    @torque_control.setter
    def torque_control(self, enable):
        if self._torque_control == enable:
            return

        self._torque_control = enable
        if enable:
            self._enable_torque_control()
        else:
            self._disable_torque_control()

    def _set_velocity_control(self, max_forces):
        pb.setJointMotorControlArray(self._bulletId,
                                     self.__dynamic_joint_indices,
                                     pb.VELOCITY_CONTROL,
                                     forces=max_forces,
                                     physicsClientId=self._client_id)

    def _enable_torque_control(self):
        """
        Please refer to the official pybullet's QuickStart Guide on Google Drive.
        By setting control mode to VELOCITY_CONTROL and max forces to zero, this
        will enable torque control.
        """
        self._set_velocity_control(np.zeros(len(self.__dynamic_joint_indices)))

    def _disable_torque_control(self):
        """
        Restore pybullet's default control mode: "VELOCITY_CONTROL" and zero
        target velocity, and set max force to a huge number (MAX_FORCE). If
        the motor is strong enough (i.e. has large enough torque upper limit),
        this is essentially locking all the motors.
        """
        self._set_velocity_control(self.q_f_max)

    def has_dynamic_joints(self):
        """Returns True, if this multi body hase any dynamic joints.
        :rtype: bool
        """
        return len(self.dynamic_joints) > 0

    @property
    def bId(self):
        """Returns the corresponding bullet Id
        :rtype: long
        """
        return self._bulletId

    def reset(self):
        """Resets this object's pose and joints to their initial configuration."""
        super().reset()
        self.apply_joint_torque_cmds(np.zeros_like(self.q))
        self.set_joint_positions(self._initial_joint_state)
        self.__last_sim_js_update = -1
        for l in self.links.values():
            l.reset()

    def get_link_index(self, link):
        if link is None:
            return -1
        else:
            if link not in self.link_index_map:
                raise Exception('Link "{}" is not defined'.format(link))
            return self.link_index_map[link]

    def get_link_by_index(self, index):
        return self.i_links[index]

    def get_link(self, link):
        return self.links[link]

    @property
    @lru_cache(1)
    def joint_names(self):
        return self.__joint_names.copy()

    @property
    @lru_cache(1)
    def dynamic_joint_names(self):
        return [self.__joint_names[x] for x in self.__dynamic_joint_indices]


    def __refresh_joint_state(self):
        if self._simulator.sim_step != self.__last_sim_js_update and len(self.__dynamic_joint_indices) > 0:
            new_js = [JointState(*x) for x in pb.getJointStates(self._bulletId, 
                                                                self.__monitored_joint_indices, 
                                                                physicsClientId=self._client_id)]
            self._q     = np.array([j.position for j in new_js[:len(self.dynamic_joints)]])
            self._q_dot = np.array([j.velocity for j in new_js[:len(self.dynamic_joints)]])
            self._q_f   = np.array([j.effort   for j in new_js[:len(self.dynamic_joints)]])
            self._joint_state = {self.__joint_names[x]: js for x, js in zip(self.__monitored_joint_indices, new_js)}        

    @property
    def q(self):
        self.__refresh_joint_state()
        return self._q

    @property
    def q_dot(self):
        self.__refresh_joint_state()
        return self._q_dot

    @property
    def q_f(self):
        self.__refresh_joint_state()
        return self._q_f

    @property
    def joint_state(self):
        """Returns the object's current joint state.
        :rtype: dict
        """
        self.__refresh_joint_state()
        return self._joint_state

    def get_ft_sensor(self, joint_name):
        if joint_name not in self.joint_sensors:
            if joint_name not in self.joints:
                raise Exception(f'Cannot get FT sensor for non-existent joint "{joint_name}"')

            pb.enableJointForceTorqueSensor(self._bulletId, 
                                            self.joints[joint_name].index, 
                                            True, 
                                            physicsClientId=self._client_id)
            self.joint_sensors[joint_name] = FTSensor(self, joint_name)
            self.__monitored_joint_indices.append(self.joints[joint_name].index)
        return self.joint_sensors[joint_name]

    def get_sensor_states(self):
        """Returns a dict of all sensors and their current readout.
        :rtype: dict
        """
        self.joint_state()

        return {sensor: self._joint_state[sensor].reactionForce for sensor in self.joint_sensors}


    def set_joint_positions(self, state, override_initial=False):
        """Sets the current joint positions of the object.

        :param pose: Joint positions to set.
        :type  pose: dict
        :param override_initial: Additionally set the given positions as initial positions.
        :type  override_initial: bool
        """
        if type(state) == dict:
            for j, p in state.items():
                if j in self.joints:
                    pb.resetJointState(self._bulletId, self.joints[j].index, p, physicsClientId=self._client_id)
            if override_initial:
                self._initial_joint_state.update(state)
        else:
            for i, p in zip(self.__dynamic_joint_indices, state):
                pb.resetJointState(self._bulletId, i, p, physicsClientId=self._client_id)
            if override_initial:
                self._initial_joint_state.update({self.__joint_names[i]: p for i, p in zip(self.__dynamic_joint_indices, state)})
        self.__last_sim_js_update = -1


    def apply_joint_pos_cmds(self, cmd, max_force=None):
        """Sets the joints' position goals.

        :param cmd: Joint position goals to set.
        :type  cmd: dict
        """
        if type(cmd) == dict:
            cmd_indices, cmd_pos = zip(*[(self.joints[j].index, c) for j, c in cmd.items() 
                                                                   if j in self.joints])
            max_force = np.take(self.q_f_max, cmd_indices) if max_force is None else max_force
        else:
            cmd_indices = self.__dynamic_joint_indices
            cmd_pos = cmd
            max_force = self.q_f_max if max_force is None else max_force

        # self.joint_driver.update_positions(self, cmd)

        self.torque_control = False


        pb.setJointMotorControlArray(self._bulletId, 
                                     cmd_indices, 
                                     pb.POSITION_CONTROL, 
                                     targetPositions=cmd_pos,
                                     targetVelocities=np.zeros_like(cmd_pos),
                                     forces=max_force,
                                     physicsClientId=self._client_id)


    def apply_joint_vel_cmds(self, cmd, max_force=None):
        """Sets the joints' velocity goals.

        :param cmd: Joint velocity goals to set.
        :type  cmd: dict
        """
        if type(cmd) == dict:
            cmd_indices, cmd_vel = zip(*[(self.joints[j].index, c) for j, c in cmd.items() 
                                                                   if j in self.joints])
        else:
            cmd_indices = self.__dynamic_joint_indices
            cmd_vel = cmd

        #print('\n'.join(['{}: {}'.format(self.__joint_names[cmd_indices[x]], cmd_vels[x]) for x in range(len(cmd_vels))])) # TODO: REMOVE THIS

        self.torque_control = False

        pb.setJointMotorControlArray(self._bulletId, 
                                     cmd_indices, 
                                     pb.VELOCITY_CONTROL, 
                                     targetVelocities=cmd_vel,
                                     forces=max_force,
                                     physicsClientId=self._client_id)

    def apply_joint_torque_cmds(self, cmd):
        """Sets the joints' torque goals.

        :param cmd: Joint effort goals to set.
        :type  cmd: dict
        """
        if type(cmd) == dict:
            cmd_indices, cmd_torque = zip(*[(self.joints[j].index, c) for j, c in cmd.items() 
                                                                   if j in self.joints])
        else:
            cmd_indices = self.__dynamic_joint_indices
            cmd_torque = cmd

        self.torque_control = True

        pb.setJointMotorControlArray(self._bulletId, 
                                     cmd_indices, 
                                     pb.TORQUE_CONTROL, 
                                     forces=cmd_torque, 
                                     physicsClientId=self._client_id)

    def get_contacts(self, other_body=None, own_link=None, other_link=None):
        """Gets the contacts this body had during the last physics step.
        The contacts can be filtered by other bodies, their links and this body's own links.

        :param other_body: Other body to filter by
        :type  other_body: MultiBody, iai_bullet_sim.rigid_body.RigidBody, NoneType
        :param own_link:   Own link to filter by
        :type  own_link:   str, NoneType
        :param other_link: Other object's link to filter by.
        :type  other_link: str, NoneType
        :rtype: list
        """
        return self._simulator.get_contacts(self, other_body, own_link, other_link)

    def get_closest_points(self, other_body=None, own_link=None, other_link=None, dist=0.2):
        """Gets the closest points of this body to its environment.
        The closest points can be filtered by other bodies, their links and this body's own links.

        :param other_body: Other body to filter by
        :type  other_body: MultiBody, iai_bullet_sim.rigid_body.RigidBody, NoneType
        :param own_link:   Own link to filter by
        :type  own_link:   str, NoneType
        :param other_link: Other object's link to filter by.
        :type  other_link: str, NoneType
        :param dist:       Maximum distance to search. Greater distance -> more expensive
        :type  dist:       float
        :rtype: list
        """
        return self._simulator.get_closest_points(self, other_body, own_link, other_link, dist)

@dataclass
class FTSensor():
    body  : MultiBody
    joint : str

    def get(self):
        self.body.joint_state

        return self.body._joint_state[self.joint].reactionForce

    def get_world_space(self):
        f_local = self.get()

        w_T_j  = self.body.joints[self.joint].pose
        f_in_w = w_T_j.dot(f_local.linear)
        t_in_w = w_T_j.dot(f_local.angular)

        return ReactionForces(f_in_w, t_in_w)
