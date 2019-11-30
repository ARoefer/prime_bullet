import pybullet as pb
from collections import namedtuple
from iai_bullet_sim.utils import Vector3, Quaternion, Frame, AABB
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
        pb.resetBasePositionAndOrientation(robot_data.bId(), (pose.position[0], pose.position[1], robot_data.initial_pos[2]), new_quat, physicsClientId=robot_data.simulator.client_id())
        pb.resetBaseVelocity(robot_data.bId(), lin_vel, ang_vel, physicsClientId=robot_data.simulator.client_id())


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
LinkState  = namedtuple('LinkState', ['CoMFrame', 'localInertialFrame', 'worldFrame', 'linearVelocity', 'angularVelocity'])


class MultiBody(object):
    """Wrapper class giving object oriented access to PyBullet's multibodies.
    """
    def __init__(self, simulator, bulletId, color, initial_pos=[0,0,0], initial_rot=[0,0,0,1], joint_driver=JointDriver(), urdf_file=None):
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
        self.simulator      = simulator
        self.__client_id    = simulator.client_id()
        self.__bulletId     = bulletId
        self.color          = color
        self.joints         = {}
        self.dynamic_joints = {}
        self.joint_idx_name_map = {}
        self.joint_sensors  = set()
        self.joint_driver   = joint_driver
        self.urdf_file      = urdf_file

        self.initial_pos    = initial_pos
        self.initial_rot    = initial_rot
        self.initial_joint_state = {}

        base_link, multibodyName = pb.getBodyInfo(self.__bulletId, physicsClientId=self.__client_id)
        #print('PyBullet says:\n  Name: {}\n  Base: {}\n'.format(multibodyName, base_link))
        self.base_link = base_link

        for x in range(pb.getNumJoints(self.__bulletId, physicsClientId=self.__client_id)):
            joint = JointInfo(*pb.getJointInfo(self.__bulletId, x, physicsClientId=self.__client_id))
            self.joints[joint.jointName] = joint
            if joint.jointType != pb.JOINT_FIXED:
                self.dynamic_joints[joint.jointName] = joint
                self.initial_joint_state[joint.jointName] = min(max(joint.lowerLimit, 0.0), joint.upperLimit)


        self.__index_joint_map       = {info.jointIndex: joint for joint, info in self.joints.items()}
        self.__dynamic_joint_indices = [info.jointIndex for info in self.dynamic_joints.values()]
        #print('dynamic joints:\n  {}'.format('\n  '.join([info.jointName for info in self.joints.values() if info.jointType != pb.JOINT_FIXED])))


        self.links = {info.linkName for info in self.joints.values()}
        self.links.add(self.base_link)
        self.link_index_map = {info.linkName: info.jointIndex for info in self.joints.values()}
        self.link_index_map[self.base_link] = -1
        self.index_link_map = {i: l for l, i in self.link_index_map.items()}

        self.__current_pose = None
        self.__last_sim_pose_update = -1
        self.__current_lin_velocity = None
        self.__current_ang_velocity = None
        self.__last_sim_velocity_update = -1

        # Initialize empty JS for objects without dynamic joints
        self.__joint_state = None if len(self.__dynamic_joint_indices) > 0 else {}
        self.__last_sim_js_update = -1


    def has_dynamic_joints(self):
        """Returns True, if this multi body hase any dynamic joints.
        :rtype: bool
        """
        return len(self.dynamic_joints) > 0

    def bId(self):
        """Returns the corresponding bullet Id
        :rtype: long
        """
        return self.__bulletId

    def register_deletion_cb(self, cb):
        """Registers a callback function which is called when this object is deleted.

        :param cb: Callback to be called. Signature f(BasicSimulator, str, RigidBody/MultiBody)
        :tyoe  cb: function
        """
        self.simulator.register_deletion_cb(self.simulator.get_body_id(self.bId()), cb)

    def deregister_deletion_cb(self, cb):
        """Deregisters a callback function which is called when this object is deleted.

        :param cb: Callback to be called. Signature f(BasicSimulator, str, RigidBody/MultiBody)
        :tyoe  cb: function
        """
        self.simulator.deregister_deletion_cb(self.simulator.get_body_id(self.bId()), cb)

    def reset(self):
        """Resets this object's pose and joints to their initial configuration."""
        pb.resetBasePositionAndOrientation(self.__bulletId, self.initial_pos, self.initial_rot, physicsClientId=self.__client_id)
        self.set_joint_positions(self.initial_joint_state)
        self.__last_sim_pose_update = -1
        self.__last_sim_js_update = -1
        self.__last_sim_velocity_update = -1

    def get_link_index(self, link):
        if link is None:
            return -1
        else:
            if link not in self.link_index_map:
                raise Exception('Link "{}" is not defined'.format(link))
            return self.link_index_map[link]

    def get_link_state(self, link=None):
        """Returns the state of the named link.
        If None is passed as link, the object's pose is returned as LinkState

        :type link: str, NoneType
        :rtype: LinkState
        """
        if link is not None and link not in self.link_index_map:
            raise Exception('Link "{}" is not defined'.format(link))

        zero_vector = Vector3(0,0,0)
        if link is None or link == self.base_link:
            frame = self.pose()
            return LinkState(frame, frame, frame, zero_vector, zero_vector)
        else:
            ls = pb.getLinkState(self.__bulletId, self.link_index_map[link], 0, physicsClientId=self.__client_id)
            return LinkState(Frame(Vector3(*ls[0]), Quaternion(*ls[1])),
                             Frame(Vector3(*ls[2]), Quaternion(*ls[3])),
                             Frame(Vector3(*ls[4]), Quaternion(*ls[5])),
                             zero_vector,
                             zero_vector)

    def get_AABB(self, linkId=None):
        """Returns the bounding box of a named link.

        :type linkId: str, NoneType
        :rtype: AABB
        """
        res = pb.getAABB(self.__bulletId, self.link_index_map[linkId], physicsClientId=self.__client_id)
        return AABB(Vector3(*res[0]), Vector3(*res[1]))

    def joint_state(self):
        """Returns the object's current joint state.
        :rtype: dict
        """
        # Avoid unnecessary updates and updates for objects without dynamic joints
        if self.simulator.get_n_update() != self.__last_sim_js_update and len(self.__dynamic_joint_indices) > 0:
            new_js = [JointState(*x) for x in pb.getJointStates(self.__bulletId, self.__dynamic_joint_indices, physicsClientId=self.__client_id)]
            self.__joint_state = {self.__index_joint_map[self.__dynamic_joint_indices[x]]: new_js[x] for x in range(len(new_js))}

        return self.__joint_state

    def pose(self):
        """Returns the object's current pose in the form of a Frame.
        :rtype: Frame
        """
        if self.simulator.get_n_update() != self.__last_sim_pose_update:
            temp = pb.getBasePositionAndOrientation(self.__bulletId, physicsClientId=self.__client_id)
            self.__current_pose = Frame(temp[0], temp[1])
        return self.__current_pose

    def linear_velocity(self):
        """Returns the object's current linear velocity.
        :rtype: list
        """
        if self.simulator.get_n_update() != self.__last_sim_velocity_update:
            self.__current_lin_velocity, self.__current_ang_velocity = pb.getBaseVelocity(self.__bulletId, physicsClientId=self.__client_id)
            self.__last_sim_velocity_update = self.simulator.get_n_update()
        return self.__current_lin_velocity

    def angular_velocity(self):
        """Returns the object's current angular velocity.
        :rtype: list
        """
        if self.simulator.get_n_update() != self.__last_sim_velocity_update:
            self.__current_lin_velocity, self.__current_ang_velocity = pb.getBaseVelocity(self.__bulletId, physicsClientId=self.__client_id)
            self.__last_sim_velocity_update = self.simulator.get_n_update()
        return self.__current_ang_velocity

    def enable_joint_sensor(self, joint_name, enable=True):
        """Activates or deactivates the force-torque sensor for a given joint.

        :type joint_name: str
        :type enable: bool
        """
        pb.enableJointForceTorqueSensor(self.__bulletId, self.joints[joint_name].jointIndex, enable, physicsClientId=self.__client_id)
        if enable:
            self.joint_sensors.add(joint_name)
        else:
            self.joint_sensors.remove(joint_name)


    def get_sensor_states(self):
        """Returns a dict of all sensors and their current readout.
        :rtype: dict
        """
        self.joint_state()

        return {sensor: self.__joint_state[sensor].reactionForce for sensor in self.joint_sensors}


    def set_pose(self, pose, override_initial=False):
        """Sets the current pose of the object.

        :param pose: Pose to set the object to.
        :type  pose: Frame
        :param override_initial: Additionally set the given pose as initial pose.
        :type  override_initial: bool
        """
        pos  = pose.position
        quat = pose.quaternion
        pb.resetBasePositionAndOrientation(self.__bulletId, pos, quat, physicsClientId=self.__client_id)
        self.__last_sim_pose_update = -1
        if override_initial:
            self.initial_pos = list(pos)
            self.initial_rot = list(quat)


    def set_joint_positions(self, state, override_initial=False):
        """Sets the current joint positions of the object.

        :param pose: Joint positions to set.
        :type  pose: dict
        :param override_initial: Additionally set the given positions as initial positions.
        :type  override_initial: bool
        """
        for j, p in state.items():
            if j in self.joints:
                pb.resetJointState(self.__bulletId, self.joints[j].jointIndex, p, physicsClientId=self.__client_id)
        self.__last_sim_js_update = -1
        if override_initial:
            self.initial_joint_state.update(state)


    def apply_joint_pos_cmds(self, cmd):
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

        pb.setJointMotorControlArray(self.__bulletId, cmd_indices, pb.POSITION_CONTROL, targetPositions=cmd_pos, physicsClientId=self.__client_id)


    def apply_joint_vel_cmds(self, cmd):
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

        pb.setJointMotorControlArray(self.__bulletId, cmd_indices, pb.VELOCITY_CONTROL, targetVelocities=cmd_vels, physicsClientId=self.__client_id)

    def apply_joint_effort_cmds(self, cmd):
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

        pb.setJointMotorControlArray(self.__bulletId, cmd_indices, pb.TORQUE_CONTROL, forces=cmd_torques, physicsClientId=self.__client_id)

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
        return self.simulator.get_contacts(self, other_body, own_link, other_link)

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
        return self.simulator.get_closest_points(self, other_body, own_link, other_link, dist)