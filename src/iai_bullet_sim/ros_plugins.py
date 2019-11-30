import rospy
import tf
import pybullet as pb
import numpy as np

from geometry_msgs.msg import WrenchStamped as WrenchMsg
from sensor_msgs.msg import JointState as JointStateMsg
from sensor_msgs.msg import LaserScan as LaserScanMsg
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryMsg
from nav_msgs.msg import Odometry as OdometryMsg

from iai_bullet_sim.basic_simulator import SimulatorPlugin
from iai_bullet_sim.multibody import MultiBody

class Watchdog(object):
    """Watchdog class. Barks if not ticked for long enough."""
    def __init__(self, timeout=rospy.Duration(0.1)):
        """Constructor. Sets time after which the dog barks."""
        self.last_tick = rospy.Time()
        self.timeout = timeout

    def tick(self, last_tick):
        """Sets the last time the dog was ticked.
        :type last_tick: rospy.Time
        """
        self.last_tick = rospy.Time.now()#last_tick

    def barks(self):
        """Returns True, if last tick was long enough ago.
        :rtype: bool
        """
        return rospy.Time.now() - self.last_tick > self.timeout


class JSPublisher(SimulatorPlugin):
    """Plugin which publishes an object's joint state to a topic.
    The joint state will be published to [prefix]/joint_states.
    """
    def __init__(self, multibody, topic_prefix=''):
        """Initializes the plugin.

        :param multibody: Object to observe.
        :type  multibody: iai_bullet_sim.multibody.MultiBody
        :param topic_prefixPrefix: for the topic to publish to.
        :type  topic_prefixPrefix: str
        """
        super(JSPublisher, self).__init__('JointState Publisher')
        self.publisher = rospy.Publisher('{}/joint_states'.format(topic_prefix), JointStateMsg, queue_size=1, tcp_nodelay=True)
        self.body = multibody
        self.topic_prefix = topic_prefix
        self.__enabled = True
        self.body.register_deletion_cb(self.on_obj_deleted)

    def on_obj_deleted(self, simulator, Id, obj):
        self.disable(simulator)
        simulator.deregister_plugin(self)

    def post_physics_update(self, simulator, deltaT):
        """Publishes the current joint state.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        if self.__enabled is False:
            return

        new_js = self.body.joint_state()
        safe_js = {}
        msg = JointStateMsg()
        msg.header.stamp = rospy.Time.now()
        for name, state in new_js.items():
            joint = self.body.joints[name]
            if (state.position < joint.lowerLimit or state.position > joint.upperLimit) and joint.upperLimit >= joint.lowerLimit:
                state.position = min(max(joint.lowerLimit, state.position), joint.upperLimit)
                safe_js[name] = state.position
            msg.name.append(name)
            msg.position.append(state.position)
            msg.velocity.append(state.velocity)
            msg.effort.append(state.effort)
        self.publisher.publish(msg)

        if len(safe_js) > 0:
            #print('Out-of bounds joints:\n  {}'.format('\n  '.join(['{}: {} {}'.format(n, self.body.joints[n].lowerLimit, self.body.joints[n].upperLimit) for n in sorted(safe_js.keys())])))
            self.body.set_joint_positions(safe_js)


    def disable(self, simulator):
        """Disables the publisher.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        """
        self.__enabled = False
        self.publisher.unregister()

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :rtype: dict
        """
        return {'body': simulator.get_body_id(self.body.bId()),
                'topic_prefix': self.topic_prefix}

    @classmethod
    def factory(cls, simulator, init_dict):
        """Instantiates the plugin from a dictionary in the context of a simulator.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type init_dict: dict
        :rtype: JSPublisher
        """
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return JSPublisher(body, init_dict['topic_prefix'])


class SensorPublisher(SimulatorPlugin):
    """Plugin which publishes an object's sensor read outs to a topic per sensor.
    The sensor data will be published to [prefix]/sensors/[sensor] as geometry_msgs/WrenchStamped.
    """
    def __init__(self, multibody, topic_prefix=''):
        """Initializes the plugin.

        :param multibody:   Object to observe.
        :type  multibody:
        :param topic_prefixPrefix: for the topics.
        :type  topic_prefixPrefix:
        """
        super(SensorPublisher, self).__init__('Sensor Publisher')
        self.publishers = {}
        for sensor in multibody.joint_sensors:
            self.publishers[sensor] = rospy.Publisher('{}/sensors/{}'.format(topic_prefix, sensor), WrenchMsg, queue_size=1, tcp_nodelay=True)
        self.body = multibody
        self.__topic_prefix = topic_prefix
        self.__enabled = True
        self.body.register_deletion_cb(self.on_obj_deleted)

    def on_obj_deleted(self, simulator, Id, obj):
        self.disable(simulator)
        simulator.deregister_plugin(self)

    def post_physics_update(self, simulator, deltaT):
        """Publishes the current sensor states.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        if self.__enabled is False:
            return

        msg = WrenchMsg()
        msg.header.stamp = rospy.Time.now()
        for sensor, state in self.body.get_sensor_states().items():
            msg.header.frame_id = self.body.joints[sensor].linkName
            msg.wrench.force.x  = state.f[0]
            msg.wrench.force.y  = state.f[1]
            msg.wrench.force.z  = state.f[2]
            msg.wrench.torque.x = state.m[0]
            msg.wrench.torque.y = state.m[1]
            msg.wrench.torque.z = state.m[2]
            if sensor not in self.publishers:
                self.publishers[sensor] = rospy.Publisher('{}/sensors/{}'.format(self.__topic_prefix, sensor), WrenchMsg, queue_size=1, tcp_nodelay=True)
            self.publishers[sensor].publish(msg)

    def disable(self, simulator):
        """Disables the publishers.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        """
        self.__enabled = False
        for publisher in self.publishers.values():
            publisher.unregister()

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :rtype: dict
        """
        return {'body': simulator.get_body_id(self.body.bId()),
                'topic_prefix': self.__topic_prefix}

    @classmethod
    def factory(cls, simulator, init_dict):
        """Instantiates the plugin from a dictionary in the context of a simulator.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type init_dict: dict
        :rtype: SensorPublisher
        """
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return SensorPublisher(body, init_dict['topic_prefix'])


class CommandSubscriber(SimulatorPlugin):
    """Superclass for plugins which subscribe to a command topic."""
    def __init__(self, name, multibody, topic, topic_type=JointStateMsg):
        """Initializes the plugin.

        :param name:        Name of the plugin.
        :type  name:
        :param multibody:   Object to observe.
        :type  multibody:
        :param topic:       Topic to subscribe to.
        :type  topic:
        :param topic_type:  Message type of the topic.
        :type  topic_type:
        """
        super(CommandSubscriber, self).__init__(name)
        self.body = multibody
        self.subscriber = rospy.Subscriber(topic, topic_type, callback=self.cmd_callback, queue_size=1)
        self._enabled = True
        self.body.register_deletion_cb(self.on_obj_deleted)

    def on_obj_deleted(self, simulator, Id, obj):
        self.disable(simulator)
        simulator.deregister_plugin(self)

    def cmd_callback(self, cmd_msg):
        """Implements the command processing behavior."""
        raise (NotImplementedError)

    def disable(self, simulator):
        """Disables the plugin and subscriber.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        """
        self._enabled = False
        self.subscriber.unregister()


class WatchdoggedJointController(CommandSubscriber):
    """Superclass for joint-level controllers.
    Instantiates a watchdog per joint and subscribes to a command topic.
    The topic is always of type sensor_msgs/JointState.
    """
    def __init__(self, name, multibody, topic, watchdog_timeout):
        """Initializes a joint-level controller, subscribing to the given topic.

        :param name:            Name of the controller
        :type  name:
        :param multibody:       Body to control
        :type  multibody:
        :param topic:           Topic to subscribe to
        :type  topic:
        :param watchdog_timeoutTimeout: for the watchdogs
        :type  watchdog_timeoutTimeout:
        """
        super(WatchdoggedJointController, self).__init__(name, multibody, topic, JointStateMsg)
        self.watchdogs = {}
        self.next_cmd  = {}
        for joint in self.body.joints.keys():
            self.watchdogs[joint] = Watchdog(rospy.Duration(watchdog_timeout))
            self.next_cmd[joint] = 0
        self.watchdog_timeout = watchdog_timeout


class JointPositionController(WatchdoggedJointController):
    """Joint-level controller which accepts joint positions as goals."""
    def __init__(self, multibody, topic_prefix='', watchdog_timeout=0.2):
        """Initializes the controller. Subscribes to [prefix]/commands/joint_positions.

        :param multibody:       Body to control
        :type  multibody:
        :param topic_prefix:    Prefix for the command topic
        :type  topic_prefix:
        :param watchdog_timeoutTimeout: for the watchdogs
        :type  watchdog_timeoutTimeout:
        """
        super(JointPositionController, self).__init__('Watchdogged Joint Position Controller', multibody, '{}/commands/joint_positions'.format(topic_prefix), watchdog_timeout)
        self.__topic_prefix = topic_prefix

    def cmd_callback(self, cmd_msg):
        """Handles the incoming command message.
        :type cmd_msg: JointStateMsg
        """
        for x in range(len(cmd_msg.name)):
            if cmd_msg.name[x] not in self.watchdogs:
                self.watchdogs[cmd_msg.name[x]] = Watchdog(rospy.Duration(self.watchdog_timeout))
            self.watchdogs[cmd_msg.name[x]].tick(cmd_msg.header.stamp)
            self.next_cmd[cmd_msg.name[x]] = cmd_msg.position[x]

    def pre_physics_update(self, simulator, deltaT):
        """Applies the current command as joint goal for the simulated joints.
        If a joint's watchdog barks, the joint is commanded to hold its position.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        if self._enabled:
            for jname in self.next_cmd.keys():
                if self.watchdogs[jname].barks():
                    self.next_cmd[jname] = self.body.joint_state()[jname].position # Stop the joint where it is

            self.body.apply_joint_pos_cmds(self.next_cmd)

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :rtype: dict
        """
        return {'body': simulator.get_body_id(self.body.bId()),
                'topic_prefix': self.__topic_prefix,
                'watchdog_timeout': self.watchdog_timeout}

    @classmethod
    def factory(cls, simulator, init_dict):
        """Instantiates the plugin from a dictionary in the context of a simulator.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type init_dict: dict
        :rtype: JointPositionController
        """
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return cls(body, init_dict['topic_prefix'], init_dict['watchdog_timeout'])


class JointVelocityController(WatchdoggedJointController):
    """Joint-level controller which accepts joint velocities as goals."""
    def __init__(self, multibody, topic_prefix='', watchdog_timeout=0.2):
        """Initializes the controller. Subscribes to [prefix]/commands/joint_velocities.

        :param multibody:       Body to control
        :type  multibody:
        :param topic_prefix:    Prefix for the command topic
        :type  topic_prefix:
        :param watchdog_timeoutTimeout: for the watchdogs
        :type  watchdog_timeoutTimeout:
        """
        super(JointVelocityController, self).__init__('Watchdogged Joint Velocity Controller', multibody, '{}/commands/joint_velocities'.format(topic_prefix), watchdog_timeout)
        self.__topic_prefix = topic_prefix

    def cmd_callback(self, cmd_msg):
        """Handles the incoming command message.
        :type cmd_msg: JointStateMsg
        """
        for x in range(len(cmd_msg.name)):
            if cmd_msg.name[x] not in self.watchdogs:
                self.watchdogs[cmd_msg.name[x]] = Watchdog(rospy.Duration(self.watchdog_timeout))
            self.watchdogs[cmd_msg.name[x]].tick(cmd_msg.header.stamp)
            self.next_cmd[cmd_msg.name[x]] = cmd_msg.velocity[x]

    def pre_physics_update(self, simulator, deltaT):
        """Applies the current command as joint goal for the simulated joints.
        If a joint's watchdog barks, the joint is commanded to stop.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        if self._enabled:
            for jname in self.next_cmd.keys():
                if self.watchdogs[jname].barks():
                    self.next_cmd[jname] = 0

            self.body.apply_joint_vel_cmds(self.next_cmd)

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :rtype: dict
        """
        return {'body': simulator.get_body_id(self.body.bId()),
                'topic_prefix': self.__topic_prefix,
                'watchdog_timeout': self.watchdog_timeout}

    @classmethod
    def factory(cls, simulator, init_dict):
        """Instantiates the plugin from a dictionary in the context of a simulator.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type init_dict: dict
        :rtype: JointVelocityController
        """
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return cls(body, init_dict['topic_prefix'], init_dict['watchdog_timeout'])


class JointVelocityDeltaContoller(JointVelocityController):
    """Joint-level controller which accepts joint velocities as goals and
    publishes the delta between those goals and the current joint velocities.
    """
    def __init__(self, multibody, topic_prefix='', watchdog_timeout=0.3):
        """Initializes the controller.
        Subscribes to [prefix]/commands/joint_velocities.
        Publishes to [prefix]/delta/joint_velocities.

        :param multibody:       Body to control
        :type  multibody:
        :param topic_prefix:    Prefix for the command topic
        :type  topic_prefix:
        :param watchdog_timeoutTimeout: for the watchdogs
        :type  watchdog_timeoutTimeout:
        """
        super(JointVelocityDeltaController, self).__init__(multibody, topic_prefix, watchdog_timeout)
        self.delta_publisher = rospy.Publisher('{}/delta/joint_velocities'.format(topic_prefix), JointStateMsg, queue_size=1, tcp_nodelay=True)

    def post_physics_update(self, simulator, deltaT):
        """Computes the delta between the given commands and
        the current joint velocities and publishes the result.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        if self._enabled:
            deltaMsg = JointStateMsg()
            deltaMsg.header.stamp = rospy.Time.now()
            js = self.body.joint_state()
            deltaMsg.position = [0]*len(self.next_cmd)
            deltaMsg.effort = deltaMsg.position
            for j, dv in self.next_cmd.items():
                deltaMsg.name.append(j)
                deltaMsg.velocity.append(dv - js[j].velocity)
            self.delta_publisher.publish(deltaMsg)


class TrajectoryPositionController(CommandSubscriber):
    """Controller which accepts a trajectory_msgs/JointTrajectory as command.
    It will pass the positional part of the trajectory on to the joints.
    It does not interpolate between trajectory points.
    """
    def __init__(self, multibody, topic_prefix=''):
        """Initializes the controller. Subscribes to [prefix]/commands/joint_trajectory.

        :param multibody:       Body to control
        :type  multibody:
        :param topic_prefix:    Prefix for the command topic
        :type  topic_prefix:
        """
        super(TrajectoryPositionController, self).__init__('Trajectory Position Controller', multibody, '{}/commands/joint_trajectory'.format(topic_prefix), JointTrajectoryMsg)
        self.trajectory = None
        self.t_start = None
        self._t_index = 0
        self.__topic_prefix = topic_prefix

    def cmd_callback(self, cmd_msg):
        """Handles the incoming command message.
        :type cmd_msg: JointTrajectoryMsg
        """
        print('Received new trajectory with {} points'.format(len(cmd_msg.points)))
        self.trajectory = []
        for point in cmd_msg.points:
            self.trajectory.append((point.time_from_start
, {cmd_msg.joint_names[x]: point.positions[x] for x in range(len(cmd_msg.joint_names))}))
        self.t_start = rospy.Time.now()
        self._t_index = 0
        self.body.set_joint_positions(self.trajectory[0][1])
        self.trajectory = sorted(self.trajectory)

    def pre_physics_update(self, simulator, deltaT):
        """Applies the current trajectory positions as commands to the joints.
        Will command the joints to remain in their last position, even after the trajectory has been fully executed.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        if self.trajectory is None or self._enabled is False:
            return

        tss = rospy.Time.now() - self.t_start
        if self.trajectory[self._t_index][0] < tss and self._t_index < len(self.trajectory) - 1:
            self._t_index += 1

        self.body.apply_joint_pos_cmds(self.trajectory[self._t_index][1])

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :rtype: dict
        """
        return {'body': simulator.get_body_id(self.body.bId()),
                'topic_prefix': self.__topic_prefix}

    @classmethod
    def factory(cls, simulator, init_dict):
        """Instantiates the plugin from a dictionary in the context of a simulator.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type init_dict: dict
        :rtype: TrajectoryPositionController
        """
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return cls(body, init_dict['topic_prefix'])


class LoopingTrajectoryPositionController(TrajectoryPositionController):
    """Controller which accepts a trajectory_msgs/JointTrajectory as command.
    It will pass the positional part of the trajectory on to the joints.
    It does not interpolate between trajectory points.
    The controller loops the trajectory execution.
    """
    def pre_physics_update(self, simulator, deltaT):
        """Applies the current trajectory positions as commands to the joints.
        Will restart trajectory execution when it is finished.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        super(LoopingTrajectoryPositionController, self).pre_physics_update(simulator, deltaT)

        if self.trajectory is not None and self._t_index is len(self.trajectory) - 1:
            self.t_start = rospy.Time.now()
            self._t_index = 0
            self.body.set_joint_positions(self.trajectory[0][1])


class ResetTrajectoryPositionController(TrajectoryPositionController):
    """Controller which accepts a trajectory_msgs/JointTrajectory as command.
    It will pass the positional part of the trajectory on to the joints.
    It does not interpolate between trajectory points.
    At the end of a trajectory execution, the controller will reset the simulator and restart the trajectory execution.
    """
    def __init__(self, simulator, multibody, topic_prefix=''):
        """Initializes the controller. Subscribes to [prefix]/commands/joint_trajectory.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type multibody: iai_bullet_sim.multibody.MultiBody
        :type topic_prefix: str
        """
        super(ResetTrajectoryPositionController, self).__init__(multibody, topic_prefix)
        self._simulator = simulator

    def pre_physics_update(self, simulator, deltaT):
        """Applies the current trajectory positions as commands to the joints.
        Will reset the simulator after the trajectory has been executed and restart trajectory execution.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        super(ResetTrajectoryPositionController, self).pre_physics_update(simulator, deltaT)

        if self.trajectory is not None and self._t_index is len(self.trajectory) - 1:
            self._simulator.reset()
            self.t_start = rospy.Time.now()
            self._t_index = 0
            self.body.set_joint_positions(self.trajectory[0][1])

    @classmethod
    def factory(cls, simulator, init_dict):
        """Instantiates the plugin from a dictionary in the context of a simulator.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type init_dict: dict
        :rtype: ResetTrajectoryPositionController
        """
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return cls(simulator, body, init_dict['topic_prefix'])


class TFPublisher(SimulatorPlugin):
    def __init__(self, simulator, map_frame='map', objects={}):
        super(TFPublisher, self).__init__('TFPublisher')
        self.map_frame = map_frame
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.objects  = objects if len(objects) > 0 else {n: o for n, o in simulator.bodies.items() if isinstance(o, MultiBody) or o.mass > 0.0}
        for obj in self.objects.values():
            obj.register_deletion_cb(self.on_obj_deleted)
        self._enabled = True

    def add_object(self, Id, obj):
        self.objects[Id] = obj
        obj.register_deletion_cb(self.on_obj_deleted)

    def on_obj_deleted(self, simulator, Id, obj):
        if Id in self.objects:
            del self.objects[Id]

    def post_physics_update(self, simulator, deltaT):
        """Implements post physics step behavior.

        :type simulator: BasicSimulator
        :type deltaT: float
        """
        if self._enabled:
            now = rospy.Time.now()
            for name, body in self.objects.items():
                pose = body.pose()
                if isinstance(body, MultiBody):
                    self.tf_broadcaster.sendTransform(pose.position, pose.quaternion, now, '{}/{}'.format(name, body.base_link), self.map_frame)
                else:
                    self.tf_broadcaster.sendTransform(pose.position, pose.quaternion, now, name, self.map_frame)

    def disable(self, simulator):
        """Stops the execution of this plugin.

        :type simulator: BasicSimulator
        """
        self._enabled = False

    @classmethod
    def factory(cls, simulator, init_dict):
        """Instantiates the plugin from a dictionary in the context of a simulator.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type init_dict: dict
        :rtype: TFPublisher
        """
        if 'objects' in init_dict:
            return cls(simulator, init_dict['map_frame'], {n: simulator.bodies[n] for n in init_dict['objects']})
        return cls(simulator, init_dict['map_frame'], {})

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.

        :type simulator: BasicSimulator
        :rtype: dict
        """
        return {'map_frame': self.map_frame, 'objects': [n for n in self.objects.keys()]}



class OdometryPublisher(SimulatorPlugin):
    """Plugin which publishes an object's current pose as nav_msgs/Odometry message."""
    def __init__(self, simulator, multibody, child_frame_id='/base_link'):
        """Initializes the plugin.

        :param simulator: Simulator
        :type  simulator: BasicSimulator
        :param multibody: Object to observe.
        :type  multibody: iai_bullet_sim.multibody.MultiBody
        :param child_frame_id: Name of the frame being published.
        :type  child_frame_id: str
        """
        super(OdometryPublisher, self).__init__('Odometry Publisher')
        name = simulator.get_body_id(multibody.bId())
        self.publisher = rospy.Publisher('{}/odometry'.format(name), OdometryMsg, queue_size=1, tcp_nodelay=True)
        self.body = multibody
        simulator.register_deletion_cb(name, self.on_obj_deleted)
        self.msg_template = OdometryMsg()
        self.msg_template.header.frame_id = 'map'
        self.msg_template.child_frame_id = child_frame_id
        self.__enabled = True

    def on_obj_deleted(self, simulator, Id, obj):
        self.disable(simulator)
        simulator.deregister_plugin(self)

    def post_physics_update(self, simulator, deltaT):
        """Publishes the current odometry.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        if self.__enabled is False:
            return

        pose = self.body.pose()
        lin_vel = self.body.linear_velocity()
        ang_vel = self.body.angular_velocity()

        self.msg_template.header.stamp = rospy.Time.now()
        self.msg_template.pose.pose.position.x = pose.position[0]
        self.msg_template.pose.pose.position.y = pose.position[1]
        self.msg_template.pose.pose.position.z = pose.position[2]
        self.msg_template.pose.pose.orientation.x = pose.quaternion[0]
        self.msg_template.pose.pose.orientation.y = pose.quaternion[1]
        self.msg_template.pose.pose.orientation.z = pose.quaternion[2]
        self.msg_template.pose.pose.orientation.w = pose.quaternion[3]

        self.msg_template.twist.twist.linear.x = lin_vel[0]
        self.msg_template.twist.twist.linear.y = lin_vel[1]
        self.msg_template.twist.twist.linear.z = lin_vel[2]
        self.msg_template.twist.twist.angular.x = ang_vel[0]
        self.msg_template.twist.twist.angular.y = ang_vel[1]
        self.msg_template.twist.twist.angular.z = ang_vel[2]
        self.publisher.publish(self.msg_template)

    def disable(self, simulator):
        """Disables the publisher.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        """
        self.__enabled = False
        self.publisher.unregister()

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :rtype: dict
        """
        return {'body': simulator.get_body_id(self.body.bId()),
                'child_frame_id': self.msg_template.child_frame_id}

    @classmethod
    def factory(cls, simulator, init_dict):
        """Instantiates the plugin from a dictionary in the context of a simulator.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type init_dict: dict
        :rtype: OdometryPublisher
        """
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return cls(simulator, body, init_dict['child_frame_id'])


def rotation3_axis_angle(axis, angle):
    """ Conversion of unit axis and angle to 4x4 rotation matrix according to:
        http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/index.htm

    :param axis: Rotation axis
    :type  axis: np.array
    :param angle: Rotation angle
    :type  angle: float
    :rtype: np.array
    """
    ct = np.cos(angle)
    st = np.sin(angle)
    vt = 1 - ct
    m_vt_0 = vt * axis[0]
    m_vt_1 = vt * axis[1]
    m_vt_2 = vt * axis[2]
    m_st_0 = axis[0] * st
    m_st_1 = axis[1] * st
    m_st_2 = axis[2] * st
    m_vt_0_1 = m_vt_0 * axis[1]
    m_vt_0_2 = m_vt_0 * axis[2]
    m_vt_1_2 = m_vt_1 * axis[2]
    return np.array([[ct + m_vt_0 * axis[0], -m_st_2 + m_vt_0_1, m_st_1 + m_vt_0_2, 0],
                      [m_st_2 + m_vt_0_1, ct + m_vt_1 * axis[1], -m_st_0 + m_vt_1_2, 0],
                      [-m_st_1 + m_vt_0_2, m_st_0 + m_vt_1_2, ct + m_vt_2 * axis[2], 0],
                      [0, 0, 0, 1]])

def rotation3_quaternion(x, y, z, w):
    """ Unit quaternion to 4x4 rotation matrix according to:
        https://github.com/orocos/orocos_kinematics_dynamics/blob/master/orocos_kdl/src/frames.cpp#L167
    """
    x2 = x * x
    y2 = y * y
    z2 = z * z
    w2 = w * w
    return np.array([[w2 + x2 - y2 - z2, 2 * x * y - 2 * w * z, 2 * x * z + 2 * w * y, 0],
                      [2 * x * y + 2 * w * z, w2 - x2 + y2 - z2, 2 * y * z - 2 * w * x, 0],
                      [2 * x * z - 2 * w * y, 2 * y * z + 2 * w * x, w2 - x2 - y2 + z2, 0],
                      [0, 0, 0, 1]])


class LaserScanner(SimulatorPlugin):
    """This class implements a virtual laser scanner for the simulation.
       The generated scans are published as sensor_msgs/LaserScan on the topic BODY/sensors/link/sensor_name.
    """
    def __init__(self, simulator, body, link,
                       ang_min, ang_max, resolution,
                       range_min, range_max, axis=(0,0,1),
                       sensor_name='laser_scan'):
        """Constructor.

        :param simulator: Simulator
        :type  simulator: BasicSimulator
        :param body: Multibody the laser scan is performed by.
        :type  body: MultiBody
        :param link: Link for the scanner
        :type  link: str
        :param ang_min: Lower edge of the laser scan.
        :type  ang_min: float
        :param ang_max: Upper edge of the laser scan.
        :type  ang_max: float
        :param resolution: Number of rays sent out by the scanner.
        :type  resolution: int
        :param range_min: Closest observable distance.
        :type  range_min: float
        :param range_max: Furthest observable distance.
        :type  range_max: float
        :param axis: rotation axis of the scanner.
        :type  axis: tuple, list
        :param sensor_name: Name of the simulated sensor.
        :type  sensor_name: str
        """
        super(LaserScanner, self).__init__('LaserScanner')
        self.body = body
        self.link = link
        self.sensor_name = sensor_name
        body_name = simulator.get_body_id(body.bId())
        simulator.register_deletion_cb(body_name, self.on_obj_deleted)

        if link is None or link == '':
            self.publisher = rospy.Publisher('{}/sensors/{}'.format(body_name, sensor_name), LaserScanMsg, queue_size=1)
        else:
            self.publisher = rospy.Publisher('{}/sensors/{}/{}'.format(body_name, link, sensor_name), LaserScanMsg, queue_size=1)
        self.resolution = resolution
        ang_step = (ang_max - ang_min) / resolution

        self.msg_template = LaserScanMsg()
        self.msg_template.header.frame_id = '{}/{}'.format(body_name, link) if link is not None else body_name
        self.msg_template.angle_min = ang_min
        self.msg_template.angle_max = ang_max
        self.msg_template.angle_increment = ang_step
        self.msg_template.range_min = range_min
        self.msg_template.range_max = range_max

        self.conversion_factor = range_max - range_min

        self.axis = axis
        e_axis = np.array([axis[0], axis[1], axis[2], 0])


        self.raw_start_points = np.hstack([rotation3_axis_angle(e_axis, ang_min + ang_step * x).dot(np.array([[range_min],[0],[0],[1]]))
                                        for x in range(resolution)]).astype(float)
        self.raw_end_points = np.hstack([rotation3_axis_angle(e_axis, ang_min + ang_step * x).dot(np.array([[range_max],[0],[0],[1]]))
                                        for x in range(resolution)]).astype(float)

        self._enabled = True

    def on_obj_deleted(self, simulator, Id, obj):
        self.disable(simulator)
        simulator.deregister_plugin(self)

    def post_physics_update(self, simulator, deltaT):
        """Performs the laser scan.

        :type simulator: BasicSimulator
        :type deltaT: float
        """
        if self._enabled:
            self.msg_template.header.stamp = rospy.Time.now()

            if self.link != None:
                ft_pose = self.body.get_link_state(self.link).worldFrame
            else:
                ft_pose = self.body.pose()

            transform = np.hstack((np.zeros((4,3)), np.array([[ft_pose.position.x],[ft_pose.position.y],[ft_pose.position.z],[0]])))
            transform = rotation3_quaternion(ft_pose.quaternion.x,
                                             ft_pose.quaternion.y,
                                             ft_pose.quaternion.z,
                                             ft_pose.quaternion.w) + transform

            start_points = np.dot(transform, self.raw_start_points).T[:,:3].tolist()
            end_points   = np.dot(transform, self.raw_end_points).T[:,:3].tolist()

            self.msg_template.ranges = [self.msg_template.range_min + r[2] * self.conversion_factor for r in pb.rayTestBatch(start_points, end_points, physicsClientId=simulator.client_id())]
            self.publisher.publish(self.msg_template)

    def disable(self, simulator):
        """Stops the execution of this plugin.

        :type simulator: BasicSimulator
        """
        self._enabled = False
        self.publisher.unregister()

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.

        :type simulator: BasicSimulator
        :rtype: dict
        """
        return {'body': simulator.get_body_id(self.body.bId()),
                'link': self.link if self.link != None else '',
                'angle_min': self.msg_template.angle_min,
                'angle_max': self.msg_template.angle_max,
                'range_min': self.msg_template.range_min,
                'range_max': self.msg_template.range_max,
                'resolution': self.resolution,
                'axis': list(self.axis)[:3],
                'sensor_name': self.sensor_name}

    @classmethod
    def factory(cls, simulator, init_dict):
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return cls(simulator, body,
                              init_dict['link'],
                              init_dict['angle_min'],
                              init_dict['angle_max'],
                              init_dict['resolution'],
                              init_dict['range_min'],
                              init_dict['range_max'],
                              init_dict['axis'],
                              init_dict['sensor_name'])
