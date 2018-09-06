import rospy
import tf

from collections import OrderedDict

from geometry_msgs.msg import WrenchStamped as WrenchMsg
from sensor_msgs.msg import JointState as JointStateMsg
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

    def post_physics_update(self, simulator, deltaT):
        """Publishes the current joint state.

        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        if self.__enabled is False:
            return

        new_js = self.body.joint_state()
        msg = JointStateMsg()
        msg.header.stamp = rospy.Time.now()
        for name, state in new_js.items():
            msg.name.append(name)
            msg.position.append(state.position)
            msg.velocity.append(state.velocity)
            msg.effort.append(state.effort)
        self.publisher.publish(msg)

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

    def cmd_callback(self, cmd_msg):
        """Implements the command processing behavior."""
        raise (NotImplemented)

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
        super(JointVelocityController, self).__init__('Watchdogged Joint Position Controller', multibody, '{}/commands/joint_positions'.format(topic_prefix), watchdog_timeout)
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
    def __init__(self, simulator, map_frame='map', objects=[]):
        super(TFPublisher, self).__init__('TFPublisher')
        self.map_frame = map_frame
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.objects  = objects if len(objects) > 0 else [(n, o) for n, o in simulator.bodies.items() if isinstance(o, MultiBody) or o.mass > 0.0]
        self._enabled = True

    def post_physics_update(self, simulator, deltaT):
        """Implements post physics step behavior.

        :type simulator: BasicSimulator
        :type deltaT: float
        """
        if self._enabled:
            now = rospy.Time.now()
            for name, body in self.objects:
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
        return cls(simulator, init_dict['map_frame'], [(n, simulator.bodies[n]) for n in init_dict['objects']])

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.

        :type simulator: BasicSimulator
        :rtype: dict
        """
        return {'map_frame': self.map_frame, 'objects': [n for n, o in self.objects]}



class OdometryPublisher(SimulatorPlugin):
    """Plugin which publishes an object's joint state to a topic.
    The joint state will be published to [prefix]/joint_states.
    """
    def __init__(self, simulator, multibody, child_frame_id='/base_link'):
        """Initializes the plugin.

        :param multibody: Object to observe.
        :type  multibody: iai_bullet_sim.multibody.MultiBody
        :param topic_prefixPrefix: for the topic to publish to.
        :type  topic_prefixPrefix: str
        """
        super(OdometryPublisher, self).__init__('Odometry Publisher')
        name = simulator.get_body_id(multibody.bId())
        self.publisher = rospy.Publisher('{}/odometry'.format(name), OdometryMsg, queue_size=1, tcp_nodelay=True)
        self.body = multibody
        self.msg_template = OdometryMsg()
        self.msg_template.header.frame_id = 'map'
        self.msg_template.child_frame_id = child_frame_id
        self.__enabled = True

    def post_physics_update(self, simulator, deltaT):
        """Publishes the current joint state.

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
        :rtype: JSPublisher
        """
        body = simulator.get_body(init_dict['body'])
        if body is None:
            raise Exception('Body "{}" does not exist in the context of the given simulation.'.format(init_dict['body']))
        return cls(simulator, body, init_dict['child_frame_id'])