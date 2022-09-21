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


class OmniBaseDriver(JointDriver):
    """Implements and update behavior for robots with a movable base.
       The base can be translated horizontally and turned around the global z-axis.
    """
    def __init__(self, max_linear_vel, max_angular_vel, x_lin_joint='base_x_joint', y_lin_joint='base_y_joint', z_ang_joint='base_angular_joint'):
        """Constructor.

        :param max_linear_vel: Upper bound for linear velocity
        :type  max_linear_vel: float
        :param max_angular_vel: Upper bound for angular velocity
        :type  max_angular_vel: float
        """
        self.m_lin_v = max_linear_vel
        self.m_ang_v = max_angular_vel
        self.x_lin_joint = x_lin_joint
        self.y_lin_joint = y_lin_joint
        self.z_ang_joint = z_ang_joint
        self.m_vel_gain = max_linear_vel
        self.m_ang_gain = max_angular_vel
        self.deltaT = 0.02

    def update_velocities(self, robot_data, velocities_dict):
        """Updates a given velocity command."""
        pose = robot_data.pose()
        lin_vel = robot_data.linear_velocity()
        ang_vel = robot_data.angular_velocity()
        inv_pos, inv_rot = pb.invertTransform(pose.position, pose.quaternion)
        ZERO_VEC = (0,0,0)
        ZERO_ROT = (0,0,0,1)
        if self.z_ang_joint in velocities_dict:
            c_ang_ib, _ = pb.multiplyTransforms(inv_pos, inv_rot, ang_vel, ZERO_ROT)
            d_ang_vel = max(min(velocities_dict[self.z_ang_joint], self.m_ang_v), -self.m_ang_v)
            ang_gain  = max(min(d_ang_vel - c_ang_ib[2], self.m_ang_gain), -self.m_ang_gain)
            del velocities_dict[self.z_ang_joint]
            ang_vel, _ = pb.multiplyTransforms(ZERO_VEC, pose.quaternion, [0.0, 0.0, c_ang_ib[2] + ang_gain], ZERO_ROT)
        else:
            ang_vel = (0,0,0)

        fwd_dir, _ = pb.multiplyTransforms(ZERO_VEC, pose.quaternion, [1,0,0], ZERO_ROT)
        yaw = atan2(fwd_dir[1], fwd_dir[0])

        if self.x_lin_joint in velocities_dict or self.y_lin_joint in velocities_dict:
            d_x_vel = 0
            d_y_vel = 0
            if self.x_lin_joint in velocities_dict:
                d_x_vel = max(min(velocities_dict[self.x_lin_joint], self.m_lin_v), -self.m_lin_v)
                del velocities_dict[self.x_lin_joint]

            if self.y_lin_joint in velocities_dict:
                d_y_vel = max(min(velocities_dict[self.y_lin_joint], self.m_lin_v), -self.m_lin_v)
                del velocities_dict[self.y_lin_joint]

            x_vel_gain = max(min(d_x_vel - lin_vel[0], self.m_vel_gain), -self.m_vel_gain)
            y_vel_gain = max(min(d_y_vel - lin_vel[1], self.m_vel_gain), -self.m_vel_gain)

            lin_vel = [lin_vel[0] + x_vel_gain, lin_vel[1] + y_vel_gain, 0] # lin_vel[2]]
        else:
            lin_vel = (0, 0, 0) #lin_vel[2])

        new_quat = pb.getQuaternionFromEuler([0,0,yaw])
        #print(' '.join(['{}'.format(type(c)) for c in list(lin_vel) + list(ang_vel)]))
        # print('New position: {}\nNew velocity: {}'.format((pose.position[0], pose.position[1], robot_data.initial_pos[2]), lin_vel))
        pb.resetBasePositionAndOrientation(robot_data.bId(), (pose.position[0], pose.position[1], robot_data.initial_pos[2]), new_quat, physicsClientId=robot_data.simulator.client_id)
        pb.resetBaseVelocity(robot_data.bId(), lin_vel, ang_vel, physicsClientId=robot_data.simulator.client_id)

    def to_dict(self):
        """Serializes the driver to a dictionary.

        :rtype: dict
        """
        return {'max_lin_vel': self.m_lin_v,
                'max_ang_vel': self.m_ang_v,
                'x_lin_joint': self.x_lin_joint,
                'y_lin_joint': self.y_lin_joint,
                'z_ang_joint': self.z_ang_joint}

    @classmethod
    def factory(cls, config_dict):
        """Instantiates the driver from a dictionary.

        :param config_dict: Driver configuration
        :type  config_dict: dict
        :rtype: SimpleBaseDriver
        """
        return SimpleBaseDriver(config_dict['max_lin_vel'], 
                                config_dict['max_ang_vel'], 
                                config_dict['x_lin_joint'],
                                config_dict['y_lin_joint'],
                                config_dict['z_ang_joint'])

class DiffDriveBaseDriver(JointDriver):
    """
    """
    def __init__(self, wheel_radius, wheel_distance, max_wheel_vel, l_wheel_joint='l_wheel_joint', r_wheel_joint='r_wheel_joint'):
        """Constructor.
        """
        self.wheel_radius   = wheel_radius
        self.wheel_distance = wheel_distance
        self.max_wheel_vel  = max_wheel_vel
        self.l_wheel_joint  = l_wheel_joint
        self.r_wheel_joint  = r_wheel_joint

    def update_velocities(self, robot_data, velocities_dict):
        """Updates a given velocity command."""
        pose = robot_data.pose()
        lin_vel = robot_data.linear_velocity()
        ang_vel = robot_data.angular_velocity()
        inv_pos, inv_rot = pb.invertTransform(pose.position, pose.quaternion)
        ZERO_VEC = (0,0,0)
        ZERO_ROT = (0,0,0,1)    

        fwd_dir, _ = pb.multiplyTransforms(ZERO_VEC, pose.quaternion, [1,0,0], ZERO_ROT)
        yaw = atan2(fwd_dir[1], fwd_dir[0])

        if self.l_wheel_joint in velocities_dict or self.r_wheel_joint in velocities_dict:
            l_wheel_vel = 0
            if self.l_wheel_joint in velocities_dict:
                l_wheel_vel = velocities_dict[self.l_wheel_joint]
                del velocities_dict[self.l_wheel_joint]
            r_wheel_vel = 0
            if self.r_wheel_joint in velocities_dict:
                r_wheel_vel = velocities_dict[self.r_wheel_joint]
                del velocities_dict[self.r_wheel_joint]
            
            # x_vel_gain = max(min(d_x_vel - lin_vel[0], self.m_vel_gain), -self.m_vel_gain)
            # y_vel_gain = max(min(d_y_vel - lin_vel[1], self.m_vel_gain), -self.m_vel_gain)

            ang_vel = (0, 0, r_wheel_vel * (self.wheel_radius / self.wheel_distance) + l_wheel_vel * (- self.wheel_radius / self.wheel_distance))
            lin_vel = [r_wheel_vel * cos(yaw) * self.wheel_radius * 0.5 + l_wheel_vel * cos(yaw) * self.wheel_radius * 0.5, 
                       r_wheel_vel * sin(yaw) * self.wheel_radius * 0.5 + l_wheel_vel * sin(yaw) * self.wheel_radius * 0.5, 0] # lin_vel[2]]
        else:
            ang_vel = (0, 0, 0)
            lin_vel = (0, 0, 0) #lin_vel[2])

        new_quat = pb.getQuaternionFromEuler([0,0,yaw])
        #print(' '.join(['{}'.format(type(c)) for c in list(lin_vel) + list(ang_vel)]))
        # print('New position: {}\nNew velocity: {}'.format((pose.position[0], pose.position[1], robot_data.initial_pos[2]), lin_vel))
        pb.resetBasePositionAndOrientation(robot_data.bId(), (pose.position[0], pose.position[1], robot_data.initial_pos[2]), new_quat, physicsClientId=robot_data.simulator.client_id)
        pb.resetBaseVelocity(robot_data.bId(), lin_vel, ang_vel, physicsClientId=robot_data.simulator.client_id)

    def to_dict(self):
        """Serializes the driver to a dictionary.

        :rtype: dict
        """
        return {'wheel_radius'   : wheel_radius,
                'wheel_distance' : wheel_distance,
                'max_wheel_vel'  : max_wheel_vel,
                'l_wheel_joint'  : l_wheel_joint,
                'r_wheel_joint'  : r_wheel_joint}

    @classmethod
    def factory(cls, config_dict):
        """Instantiates the driver from a dictionary.

        :param config_dict: Driver configuration
        :type  config_dict: dict
        :rtype: SimpleBaseDriver
        """
        return cls(config_dict['wheel_radius'], 
                   config_dict['wheel_distance'], 
                   config_dict['max_wheel_vel'],
                   config_dict['l_wheel_joint'],
                   config_dict['r_wheel_joint'])


class SimpleBaseDriver(JointDriver):
    """Implements and update behavior for robots with a movable base.
       The base can be translated horizontally and turned around the global z-axis.
    """
    def __init__(self, max_linear_vel, max_angular_vel, x_lin_joint='base_linear_joint', y_lin_joint='base_perp_joint', z_ang_joint='base_angular_joint'):
        """Constructor.

        :param max_linear_vel: Upper bound for linear velocity
        :type  max_linear_vel: float
        :param max_angular_vel: Upper bound for angular velocity
        :type  max_angular_vel: float
        """
        self.m_lin_v = max_linear_vel
        self.m_ang_v = max_angular_vel
        self.x_lin_joint = x_lin_joint
        self.y_lin_joint = y_lin_joint
        self.z_ang_joint = z_ang_joint
        self.m_vel_gain = max_linear_vel
        self.m_ang_gain = max_angular_vel
        self.deltaT = 0.02

    def update_velocities(self, robot_data, velocities_dict):
        """Updates a given velocity command."""
        pose = robot_data.pose()
        lin_vel = robot_data.linear_velocity()
        ang_vel = robot_data.angular_velocity()
        inv_pos, inv_rot = pb.invertTransform(pose.position, pose.quaternion)
        ZERO_VEC = (0,0,0)
        ZERO_ROT = (0,0,0,1)
        if self.z_ang_joint in velocities_dict:
            c_ang_ib, _ = pb.multiplyTransforms(inv_pos, inv_rot, ang_vel, ZERO_ROT)
            d_ang_vel = max(min(velocities_dict[self.z_ang_joint], self.m_ang_v), -self.m_ang_v)
            ang_gain  = max(min(d_ang_vel - c_ang_ib[2], self.m_ang_gain), -self.m_ang_gain)
            del velocities_dict[self.z_ang_joint]
            ang_vel, _ = pb.multiplyTransforms(ZERO_VEC, pose.quaternion, [0.0, 0.0, c_ang_ib[2] + ang_gain], ZERO_ROT)
        else:
            ang_vel = (0,0,0)

        fwd_dir, _ = pb.multiplyTransforms(ZERO_VEC, pose.quaternion, [1,0,0], ZERO_ROT)
        yaw = atan2(fwd_dir[1], fwd_dir[0])

        if self.x_lin_joint in velocities_dict or self.y_lin_joint in velocities_dict:
            c_vel_ib, _ = pb.multiplyTransforms(ZERO_VEC, inv_rot, lin_vel, ZERO_ROT)

            d_fwd_vel = 0
            d_strafe_vel = 0
            if self.x_lin_joint in velocities_dict:
                d_fwd_vel = max(min(velocities_dict[self.x_lin_joint], self.m_lin_v), -self.m_lin_v)
                del velocities_dict[self.x_lin_joint]

            if self.y_lin_joint in velocities_dict:
                d_strafe_vel = max(min(velocities_dict[self.y_lin_joint], self.m_lin_v), -self.m_lin_v)
                del velocities_dict[self.y_lin_joint]

            fwd_vel_gain = max(min(d_fwd_vel - c_vel_ib[0], self.m_vel_gain), -self.m_vel_gain)
            strafe_vel_gain = max(min(d_strafe_vel - c_vel_ib[1], self.m_vel_gain), -self.m_vel_gain)


            cos_sq = cos(ang_vel[2] * self.deltaT) ** 2
            sin_sq = sin(ang_vel[2] * self.deltaT) ** 2
            n_fwd_vel    = cos_sq * (c_vel_ib[0] + fwd_vel_gain) + sin_sq * (c_vel_ib[1] + strafe_vel_gain)
            n_strafe_vel = sin_sq * (c_vel_ib[0] + fwd_vel_gain) + cos_sq * (c_vel_ib[1] + strafe_vel_gain)
            d_lin_vel, _ = pb.multiplyTransforms(ZERO_VEC, pose.quaternion, [n_fwd_vel, n_strafe_vel, 0], ZERO_ROT)
            lin_vel = [d_lin_vel[0], d_lin_vel[1], lin_vel[2]]
        else:
            lin_vel = (0, 0, lin_vel[2])

        new_quat = pb.getQuaternionFromEuler([0,0,yaw])
        #print(' '.join(['{}'.format(type(c)) for c in list(lin_vel) + list(ang_vel)]))
        pb.resetBasePositionAndOrientation(robot_data.bId(), 
                                           (pose.position[0], pose.position[1], robot_data.initial_pos[2]), 
                                           new_quat, 
                                           physicsClientId=robot_data.simulator.client_id)
        pb.resetBaseVelocity(robot_data.bId(), 
                             lin_vel, 
                             ang_vel, 
                             physicsClientId=robot_data.simulator.client_id)


    def to_dict(self):
        """Serializes the driver to a dictionary.

        :rtype: dict
        """
        return {'max_lin_vel': self.m_lin_v,
                'max_ang_vel': self.m_ang_v,
                'x_lin_joint': self.x_lin_joint,
                'y_lin_joint': self.y_lin_joint,
                'z_ang_joint': self.z_ang_joint}

    @classmethod
    def factory(cls, config_dict):
        """Instantiates the driver from a dictionary.

        :param config_dict: Driver configuration
        :type  config_dict: dict
        :rtype: SimpleBaseDriver
        """
        return SimpleBaseDriver(config_dict['max_lin_vel'], 
                                config_dict['max_ang_vel'], 
                                config_dict['x_lin_joint'],
                                config_dict['y_lin_joint'],
                                config_dict['z_ang_joint'])



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
        self.reactionForce = ReactionForces(rForce[:3], rForce[3:])
        self.effort = appliedTorque

# Reaction force structure. Assigns names to bullet's info structure.
ReactionForces  = namedtuple('ReactionForces', ['f', 'm'])

# Joint info structure. Assigns names to bullet's info structure.
JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType', 'qIndex', 'uIndex',
                                     'flags', 'jointDamping', 'jointFriction', 'lowerLimit',
                                     'upperLimit', 'maxEffort', 'maxVelocity', 'linkName',
                                     'axis', 'parentFramePos', 'parentFrameOrn', 'parentIndex'])


# Link state structure. Assigns names to bullet's info structure.
@dataclass
class LinkState:
    com_pose            : Transform
    local_inertial_pose : Transform
    world_pose          : Transform
    linear_velocity     : Vector3
    angular_velocity    : Vector3


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

        for x in range(pb.getNumJoints(self._bulletId, physicsClientId=self._client_id)):
            joint = JointInfo(*pb.getJointInfo(self._bulletId, x, physicsClientId=self._client_id))
            self.joints[joint.jointName.decode('utf-8')] = joint
            if joint.jointType != pb.JOINT_FIXED:
                self.dynamic_joints[joint.jointName.decode('utf-8')] = joint
                self._initial_joint_state[joint.jointName.decode('utf-8')] = min(max(joint.lowerLimit, 0.0), joint.upperLimit)


        self.__index_joint_map       = {info.jointIndex: joint for joint, info in self.joints.items()}
        self.__dynamic_joint_indices = [info.jointIndex for info in self.dynamic_joints.values()]
        #print('dynamic joints:\n  {}'.format('\n  '.join([info.jointName for info in self.joints.values() if info.jointType != pb.JOINT_FIXED])))


        self.links = {info.linkName.decode('utf-8') for info in self.joints.values()}
        self.links.add(self.base_link)
        self.link_index_map = {info.linkName.decode('utf-8'): info.jointIndex for info in self.joints.values()}
        self.link_index_map[self.base_link] = -1
        self.links = {n: Link(self._simulator, self, i) for n, i in self.link_index_map.items()}
        self.index_link_map = {i: l for l, i in self.link_index_map.items()}

        # Initialize empty JS for objects without dynamic joints
        self.__joint_state = None if len(self.__dynamic_joint_indices) > 0 else {}
        self.__last_sim_js_update = -1


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

    def get_link(self, link):
        return self.links[link]

    @property
    @lru_cache(1)
    def joint_names(self):
        return self.__dynamic_joint_indices.copy()

    @property
    def joint_state(self):
        """Returns the object's current joint state.
        :rtype: dict
        """
        # Avoid unnecessary updates and updates for objects without dynamic joints
        if self._simulator.sim_step != self.__last_sim_js_update and len(self.__dynamic_joint_indices) > 0:
            new_js = [JointState(*x) for x in pb.getJointStates(self._bulletId, self.__dynamic_joint_indices, physicsClientId=self._client_id)]
            self.__joint_state = {self.__index_joint_map[self.__dynamic_joint_indices[x]]: new_js[x] for x in range(len(new_js))}

        return self.__joint_state

    def get_ft_sensor(self, joint_name):
        if joint_name not in self.joint_sensors:
            if joint_name not in self.joints:
                raise Exception(f'Cannot get FT sensor for non-existent joint "{joint_name}"')

            pb.enableJointForceTorqueSensor(self._bulletId, 
                                            self.joints[joint_name].jointIndex, 
                                            True, 
                                            physicsClientId=self._client_id)
            self.joint_sensors[joint_name] = FTSensor(self, joint_name)
        return self.joint_sensors[joint_name]

    def get_sensor_states(self):
        """Returns a dict of all sensors and their current readout.
        :rtype: dict
        """
        self.joint_state()

        return {sensor: self.__joint_state[sensor].reactionForce for sensor in self.joint_sensors}


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
                    pb.resetJointState(self._bulletId, self.joints[j].jointIndex, p, physicsClientId=self._client_id)
            if override_initial:
                self._initial_joint_state.update(state)
        else:
            for i, p in enumerate(state):
                pb.resetJointState(self._bulletId, i, p, physicsClientId=self._client_id)
            if override_initial:
                self._initial_joint_state.update({self.__index_joint_map[i]: x for i, x in enumerate(state)})
        self.__last_sim_js_update = -1


    def apply_joint_pos_cmds(self, cmd, max_force=None):
        """Sets the joints' position goals.

        :param cmd: Joint position goals to set.
        :type  cmd: dict
        """
        cmd_indices = []
        cmd_pos     = []
        self.joint_driver.update_positions(self, cmd)
        for jname in cmd.keys():
            if jname in self.joints:
                cmd_indices.append(self.joints[jname].jointIndex)
                cmd_pos.append(cmd[jname])

        pb.setJointMotorControlArray(self._bulletId, 
                                     cmd_indices, 
                                     pb.POSITION_CONTROL, 
                                     targetPositions=cmd_pos,
                                     forces=max_force,
                                     physicsClientId=self._client_id)


    def apply_joint_vel_cmds(self, cmd, max_force=None):
        """Sets the joints' velocity goals.

        :param cmd: Joint velocity goals to set.
        :type  cmd: dict
        """
        cmd_indices = []
        cmd_vels    = []
        self.joint_driver.update_velocities(self, cmd)
        for jname in cmd.keys():
            if jname in self.joints:
                cmd_indices.append(self.joints[jname].jointIndex)
                cmd_vels.append(cmd[jname])

        #print('\n'.join(['{}: {}'.format(self.__index_joint_map[cmd_indices[x]], cmd_vels[x]) for x in range(len(cmd_vels))])) # TODO: REMOVE THIS

        pb.setJointMotorControlArray(self._bulletId, 
                                     cmd_indices, 
                                     pb.VELOCITY_CONTROL, 
                                     targetVelocities=cmd_vels,
                                     forces=max_force,
                                     physicsClientId=self._client_id)

    def apply_joint_torque_cmds(self, cmd):
        """Sets the joints' torque goals.

        :param cmd: Joint effort goals to set.
        :type  cmd: dict
        """
        cmd_indices = []
        cmd_torques = []
        self.joint_driver.update_effort(self, cmd)
        for jname in cmd.keys():
            if jname in self.joints:
                cmd_indices.append(self.joints[jname].jointIndex)
                cmd_torques.append(cmd[jname])

        pb.setJointMotorControlArray(self._bulletId, 
                                     cmd_indices, 
                                     pb.TORQUE_CONTROL, 
                                     forces=cmd_torques, 
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


class Link(Frame):
    def __init__(self, simulator, multibody : MultiBody, idx : int):
        super().__init__(None)

        self._simulator = simulator
        self._client_id = simulator.client_id
        self._multibody = multibody
        self._idx       = idx

        self.__current_state = None
        self.__last_sim_pose_update = -1
        self.__current_aabb = None
        self.__last_aabb_update = -1

    @property
    def idx(self):
        return self._idx

    @property
    def local_pose(self) -> Transform:
        return self.state.world_pose
    
    @property
    def state(self):
        if self._simulator.sim_step != self.__last_sim_pose_update:
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

    def ik(self, world_pose : Union[Point3, Transform]):
        if type(world_pose) == Point3:
            return np.asarray(pb.calculateInverseKinematics(self._multibody.bId,
                                                            self._idx,
                                                            world_pose,
                                                            physicsClientId=self._client_id
                                                            ))
        return np.asarray(pb.calculateInverseKinematics(self._multibody.bId,
                                                            self._idx,
                                                            world_pose.position,
                                                            world_pose.quaternion,
                                                            physicsClientId=self._client_id
                                                            ))

@dataclass
class FTSensor():
    body  : MultiBody
    joint : str

    def get(self):
        self.body.joint_state()

        return self.body.__joint_state[self.joint].reactionForce

