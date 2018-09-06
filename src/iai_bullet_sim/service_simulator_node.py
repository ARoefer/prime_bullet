import rospy
import traceback

from multiprocessing import Lock

from geometry_msgs.msg import Pose, Vector3
from visualization_msgs.msg import Marker as MarkerMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.basic_simulator_node import BasicSimulatorNode
from iai_bullet_sim.rigid_body  import RigidBody
from iai_bullet_sim.multibody   import MultiBody
from iai_bullet_sim.msg import MultiBody as MultiBodyMsg
from iai_bullet_sim.msg import RigidBody as RigidBodyMsg
from iai_bullet_sim.ros_plugins import JSPublisher, SensorPublisher, JointPositionController, JointVelocityController, JointVelocityDeltaContoller, TrajectoryPositionController, LoopingTrajectoryPositionController, ResetTrajectoryPositionController
from iai_bullet_sim.srv import *
from iai_bullet_sim.utils import Frame

controller_map = {SetControllerRequest.TYPE_VELOCITY: JointVelocityController,
                      SetControllerRequest.TYPE_POSITION: JointPositionController,
                      SetControllerRequest.TYPE_VELDELTA: JointVelocityDeltaContoller,
                      SetControllerRequest.TYPE_TRAJECTORY: TrajectoryPositionController,
                      SetControllerRequest.TYPE_TRAJECTORY_LOOP: LoopingTrajectoryPositionController,
                      SetControllerRequest.TYPE_TRAJECTORY_RESET: ResetTrajectoryPositionController}

class ServiceSimulatorNode(BasicSimulatorNode):
    """This class exposes a lot of the simulator's functionality to ROS services."""
    def __init__(self, simulator_class=BasicSimulator):
        super(ServiceSimulatorNode, self).__init__(simulator_class)
        self.services =[
            rospy.Service('get_object_ids', GetObjectIds, self.srv_get_obj_ids),
            rospy.Service('get_rigid_object_ids', GetObjectIds, self.srv_get_rigid_ids),
            rospy.Service('get_multibody_ids', GetObjectIds, self.srv_get_multibody_ids),
            rospy.Service('get_joints', GetJoints, self.srv_get_obj_joints),
            rospy.Service('get_multibodies', GetMultibodies, self.srv_get_multibodies),
            rospy.Service('get_rigidbodies', GetRigidBodies, self.srv_get_rigidbodies),
            rospy.Service('add_rigid_object', AddRigidObject, self.srv_add_rigid_body),
            rospy.Service('add_urdf', AddURDF, self.srv_add_urdf),
            rospy.Service('add_controller', SetController, self.srv_add_controller),
            rospy.Service('remove_controller', SetController, self.srv_remove_controller),
            rospy.Service('set_joint_sensor', SetJointSensor, self.srv_set_joint_sensor),
            rospy.Service('set_joint_state', SetJointState, self.srv_set_joint_state),
            rospy.Service('set_object_pose', SetObjectPose, self.srv_set_pose),
            rospy.Service('set_gravity', SetGravity, self.srv_set_gravity),
            rospy.Service('save_to_yaml', SaveSimulator, self.srv_save_to_yaml),
            rospy.Service('save_to_rosparam', SaveSimulator, self.srv_save_to_rosparam),
            rospy.Service('reset_object', ObjectId, self.srv_reset_object),
            rospy.Service('reset', Empty, self.srv_reset),
            rospy.Service('pause', Empty, self.srv_pause),
            rospy.Service('run',   Empty, self.srv_run),
            rospy.Service('get_simulator_state', GetSimulatorState, self.srv_get_state),
            rospy.Service('get_static_geometry', GetStaticGeometry, self.srv_get_static_geometry)
        ]
        self.lock = Lock()
        self.sensor_publishers = {}
        self.controllers = {}

    def tick(self, timer_event):
        with self.lock:
            super(ServiceSimulatorNode, self).tick(timer_event)

    def srv_get_obj_ids(self, req):
        with self.lock:
            res = GetObjectIdsResponse()
            res.object_ids = self.sim.bodies.keys()
            return res

    def srv_get_rigid_ids(self, req):
        with self.lock:
            res = GetObjectIdsResponse()
            res.object_ids = [k for k, v in self.sim.bodies.items() if isinstance(v, RigidBody)]
            return res

    def srv_get_multibody_ids(self, req):
        with self.lock:
            res = GetObjectIdsResponse()
            res.object_ids = [k for k, v in self.sim.bodies.items() if isinstance(v, MultiBody)]
            return res

    def srv_get_multibodies(self, req):
        with self.lock:
            res = GetMultibodiesResponse()
            
            for name, body in self.sim.bodies.items():
                if isinstance(body, MultiBody):
                    msg = MultiBodyMsg()
                    msg.joints       = body.dynamic_joints.keys()
                    msg.links        = list(body.links)
                    msg.sensors      = list(body.joint_sensors)
                    msg.urdf_file    = body.urdf_file
                    msg.initial_pose = Pose()
                    for a, v in zip(['x', 'y', 'z'], body.initial_pos):
                        setattr(msg.initial_pose.position, a, v)
                    for a, v in zip(['x', 'y', 'z', 'w'], body.initial_pos):
                        setattr(msg.initial_pose.orientation, a, v)
                    msg.initial_joint_state = JointState()
                    for j, v in body.initial_joint_state.items():
                        msg.initial_joint_state.name.append(j)
                        msg.initial_joint_state.position.append(v)
                        msg.initial_joint_state.velocity.append(0)
                        msg.initial_joint_state.effort.append(0)
                    res.name.append(name)
                    res.body.append(msg)
                    if name in self.controllers:
                        res.controller.extend(self.controllers[name].keys())
                        res.controller.extend([name]*len(self.controllers[name]))

            return res

    def srv_get_rigidbodies(self, req):
        with self.lock:
            res = GetRigidBodiesResponse()
            
            for name, body in self.sim.bodies.items():
                if isinstance(body, RigidBody):
                    msg = RigidBodyMsg()
                    msg.geom_type = body.type
                    for a, v in zip(['x', 'y', 'z'], body.extents):
                        setattr(msg.extents, a, v)
                    msg.height  = body.height
                    msg.radius  = body.radius
                    msg.mass    = body.mass 
                    for a, v in zip(['r', 'g', 'b', 'a'], body.color):
                        setattr(msg.color, a, v)
                    for a, v in zip(['x', 'y', 'z'], body.initial_pos):
                        setattr(msg.initial_pose.position, a, v)
                    for a, v in zip(['x', 'y', 'z', 'w'], body.initial_pos):
                        setattr(msg.initial_pose.orientation, a, v)
                    res.name.append(name)
                    res.body.append(msg)

            return res

    def srv_get_obj_joints(self, req):
        with self.lock:
            res = GetJointsResponse()
            if req.object_id in self.sim.bodies:
                body = self.sim.bodies[req.object_id]
                if isinstance(body, MultiBody):
                    res.joints  = body.dynamic_joints.keys()
                    res.success = True
                    return res
                else:
                    res.error_msg = '{} is not a multibody.'.format(req.object_id)
            else:
                res.error_msg = '{} is not a known object.'.format(req.object_id)
            res.success = False
            return res

    def srv_add_rigid_body(self, req):
        with self.lock:
            res   = AddRigidObjectResponse()
            name  = req.name if req.name is not '' else None
            pos   = [req.pose.position.x, req.pose.position.y, req.pose.position.z]
            quat  = [req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w]
            extents = [req.extents.x, req.extents.y, req.extents.z]
            color = [req.color.r, req.color.g, req.color.b, req.color.a] if req.color.a > 0 else None
            try:
                body = self.sim.create_object(req.geom_type, extents, req.radius, req.height, pos, quat, req.mass, color, name)
                res.success   = True
                res.object_id = self.sim.get_body_id(body.bId())
            except Exception as e:
                traceback.print_exc()
                res.success = False
                res.error_msg = str(e)

            return res

    def srv_add_urdf(self, req):
        with self.lock:
            res   = AddURDFResponse()
            name  = req.name if req.name is not '' else None
            pos   = [req.pose.position.x, req.pose.position.y, req.pose.position.z]
            quat  = [req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w]
            try:
                body = self.sim.load_urdf(req.urdf_path, pos, quat, useFixedBase=req.fixed_base, name_override=name)
                res.success   = True
                res.object_id = self.sim.get_body_id(body.bId())

                if body.has_dynamic_joints():
                    self.sim.register_plugin(JSPublisher(body, res.object_id))
            except Exception as e:
                traceback.print_exc()
                res.success = False
                res.error_msg = str(e)

            return res

    def srv_add_controller(self, req):
        with self.lock:
            res = SetControllerResponse()
            res.success = False
            body = self.sim.get_body(req.object_id)
            if body is None:
                res.error_msg = 'Object {} does not exist.'.format(req.object_id)
                return res

            if not isinstance(body, MultiBody):
                res.error_msg = '{} is not a multi body.'.format(req.object_id)
                return res

            if not body.has_dynamic_joints():
                res.error_msg = '{} has no dynamic joint.'.format(req.object_id)
                return res

            if req.controller_type not in controller_map:
                res.error_msg = 'Controller type {} is unknown.'.format(req.controller_type)
                return res

            if req.object_id in self.controllers:
                if req.controller_type in self.controllers[req.object_id]:
                    res.success = True
                    return res
            else:
                self.controllers[req.object_id] = {}

            if req.controller_type != SetControllerRequest.TYPE_TRAJECTORY_RESET:
                controller = controller_map[req.controller_type](body, req.object_id)
            else:
                controller = ResetTrajectoryPositionController(self.sim, body, req.object_id)
            self.sim.register_plugin(controller)
            self.controllers[req.object_id][req.controller_type] = controller
            res.success = True
            return res

    def srv_remove_controller(self, req):
        with self.lock:
            res = SetControllerResponse()
            res.success = False
            if req.object_id in self.controllers:
                if req.controller_type in self.controllers[req.object_id]:
                    res.success = True

                    controller = self.controllers[req.object_id][req.controller_type]
                    controller.disable(self.sim)
                    self.sim.deregister_plugin(controller)
                    del self.controllers[req.object_id][req.controller_type]
                    return res
                else:
                    res.error_msg = '{} does not have a controller of type {}.'.format(req.object_id, req.controller_type)
            else:
                res.error_msg = '{} does not have any controllers.'.format(req.object_id)
            return res

    def srv_set_joint_sensor(self, req):
        with self.lock:
            res  = SetJointSensorResponse()
            res.success = False
            body = self.sim.get_body(req.object_id)
            if body is None:
                res.error_msg = 'Object {} does not exist.'.format(req.object_id)
                return res

            if not isinstance(body, MultiBody):
                res.error_msg = '{} is not a multi body.'.format(req.object_id)
                return res

            if req.joint not in body.joints:
                res.error_msg = '{} does not have a joint called {}.'.format(req.object_id, req.joint)
                return res

            body.enable_joint_sensor(req.joint, req.enable)
            if req.enable:
                if req.object_id not in self.sensor_publishers:
                    self.sensor_publishers[req.object_id] = SensorPublisher(body, req.object_id)
                    self.sim.register_plugin(self.sensor_publishers[req.object_id])
            else:
                if req.object_id in self.sensor_publishers and len(body.joint_sensors) == 0:
                    self.sensor_publishers[req.object_id].disable(self.sim)
                    self.sim.deregister_plugin(self.sensor_publishers[req.object_id])
                    del self.sensor_publishers[req.object_id]

            res.success = True
            return res

    def srv_set_joint_state(self, req):
        with self.lock:
            res  = SetJointStateResponse()
            res.success = False
            body = self.sim.get_body(req.object_id)
            if body is None:
                res.error_msg = 'Object {} does not exist.'.format(req.object_id)
                return res

            if not isinstance(body, MultiBody):
                res.error_msg = '{} is not a multi body.'.format(req.object_id)
                return res

            try:
                body.set_joint_positions({req.joint_state.name[x]: req.joint_state.position[x] for x in range(len(req.joint_state.name))}, req.override_initial)
                res.success = True
                return res
            except Exception as e:
                traceback.print_exc()
                res.error_msg = str(e)
            return res

    def srv_set_pose(self, req):
        with self.lock:
            res  = SetObjectPoseResponse()
            res.success = False
            body = self.sim.get_body(req.object_id)
            if body is None:
                res.error_msg = 'Object {} does not exist.'.format(req.object_id)
                return res

            try:
                pos   = [req.pose.position.x, req.pose.position.y, req.pose.position.z]
                quat  = [req.pose.orientation.x, req.pose.orientation.y, req.pose.orientation.z, req.pose.orientation.w]
                body.set_pose(Frame(pos, quat), req.override_initial)
                res.success = True
                return res
            except Exception as e:
                traceback.print_exc()
                res.error_msg = str(e)
            return res

    def srv_save_to_yaml(self, req):
        with self.lock:
            res = SaveSimulatorResponse()
            res.success = False
            try:
                self.save_to_yaml(req.path, req.use_current_as_initial)
                res.success = True
            except Exception as e:
                traceback.print_exc()
                res.error_msg = str(e)
            return res

    def srv_save_to_rosparam(self, req):
        with self.lock:
            res = SaveSimulatorResponse()
            res.success = False
            try:
                self.save_to_rosparam(req.path, req.use_current_as_initial)
                res.success = True
            except Exception as e:
                traceback.print_exc()
                res.error_msg = str(e)
            return res

    def srv_load_from_yaml(self, req):
        with self.lock:
            pass

    def srv_load_from_rosparam(self, req):
        with self.lock:
            pass

    def srv_reset_object(self, req):
        with self.lock:
            res = ObjectIdResponse()
            res.success = False
            body = self.sim.get_body(req.object_id)
            if body is None:
                res.error_msg = 'Object {} does not exist.'.format(req.object_id)
                return res

            body.reset()

            res.success = True
            return res

    def srv_reset(self, req):
        with self.lock:
            res = EmptyResponse()
            res.success = False
            try:
                self.sim.reset()
                res.success = True
            except Exception as e:
                traceback.print_exc()
                res.error_msg = str(e)
            return res

    def srv_set_gravity(self, req):
        with self.lock:
            res = SetGravityResponse()
            res.success = False
            try:
                self.sim.set_gravity([req.gravity.x, req.gravity.y, req.gravity.z])
                res.success = True
            except Exception as e:
                traceback.print_exc()
                res.error_msg = str(e)
            return res

    def srv_pause(self, msg):
        with self.lock:
            res = EmptyResponse()
            res.success = False
            try:
                self.pause()
                res.success = True
            except Exception as e:
                traceback.print_exc()
                res.error_msg = str(e)
            return res

    def srv_run(self, msg):
        with self.lock:
            res = EmptyResponse()
            res.success = False
            try:
                self.run()
                res.success = True
            except Exception as e:
                traceback.print_exc()
                res.error_msg = str(e)
            return res

    def srv_get_state(self, msg):
        with self.lock:
            res = GetSimulatorStateResponse()
            if self.is_running():
                res = GetSimulatorStateResponse.RUNNING
            else:
                res = GetSimulatorStateResponse.PAUSED
            return res

    def srv_get_static_geometry(self, msg):
        with self.lock:
            res = GetStaticGeometryResponse()
            for name, body in self.sim.bodies.items():
                if isinstance(body, RigidBody) and body.mass <= 0.0:
                    res.geometry.extend(rigid_body_to_markers(body, name))

            res.success = True
            return res


def rigid_body_to_markers(body, name):
    central_marker = MarkerMsg()
    central_marker.ns = name
    out = [central_marker]
    pos, quat = body.pose()

    for a, v in zip(['x', 'y', 'z'], pos):
        setattr(central_marker.pose.position, a, v)

    for a, v in zip(['x', 'y', 'z', 'w'], quat):
        setattr(central_marker.pose.orientation, a, v)

    for a, v in zip(['r', 'g', 'b', 'a'], body.color):
        setattr(central_marker.color, a, v)
    if body.type == 'box':
        central_marker.type = MarkerMsg.CUBE
        for a, v in zip(['x', 'y', 'z'], body.extents):
            setattr(central_marker.scale, a, v)
    elif body.type == 'sphere':
        central_marker.type = MarkerMsg.SPHERE
        for a in ['x', 'y', 'z']:
            setattr(central_marker.scale, a, body.radius * 2)
    elif body.type == 'cylinder' or body.type == 'capsule':
        central_marker.type = MarkerMsg.CYLINDER
        for a in ['x', 'y']:
            setattr(central_marker.scale, a, body.radius * 2)
        # if body.type == 'capsule':
        #     central_marker.scale.z = body.height - 2 * body.radius 
        #     upper_sphere = MarkerMsg()
        #     upper_sphere.type  = MarkerMsg.SPHERE
        #     upper_sphere.color = central_marker.color
        #     upper_sphere.scale.x = body.radius
        #     upper_sphere.pose.position.z = body.height * 0.5 - body.radius
        #     upper_sphere.pose.orientation.w = 1
        #     lower_sphere = MarkerMsg()
        #     lower_sphere.type  = MarkerMsg.SPHERE
        #     lower_sphere.color = central_marker.color
        #     lower_sphere.scale.x = body.radius
        #     lower_sphere.pose.position.z = -body.height * 0.5 + body.radius
        #     lower_sphere.pose.orientation.w = 1
        #     out.append(lower_sphere)
        #     out.append(upper_sphere)
        # else:
        #     central_marker.scale.z = body.height

    return out