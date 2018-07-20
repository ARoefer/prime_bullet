import rospy

from iai_bullet_sim.basic_simulator_node import BasicSimulatorNode
from iai_bullet_sim.rigid_body  import RigidBody
from iai_bullet_sim.multibody   import MultiBody
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
    def __init__(self):
        self.services =[
            rospy.Service('get_object_ids', GetObjectIds, self.srv_get_obj_ids),
            rospy.Service('get_rigid_object_ids', GetObjectIds, self.srv_get_rigid_ids),
            rospy.Service('get_multibody_ids', GetObjectIds, self.srv_get_multibody_ids),
            rospy.Service('get_joints', GetJoints, self.srv_get_obj_joints),
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
            rospy.Service('reset', Empty, self.srv_reset)
        ]
        self.sensor_publishers = {}
        self.controllers = {}

    def srv_get_obj_ids(self, req):
        res = GetObjectIdsResponse()
        res.object_ids = self.sim.bodies.keys()
        return res

    def srv_get_rigid_ids(self, req):
        res = GetObjectIdsResponse()
        res.object_ids = [k for k, v in self.sim.bodies.items() if isinstance(v, RigidBody)]
        return res

    def srv_get_multibody_ids(self, req):
        res = GetObjectIdsResponse()
        res.object_ids = [k for k, v in self.sim.bodies.items() if isinstance(v, MultiBody)]
        return res

    def srv_get_obj_joints(self, req):
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
            res.success = False
            res.error_msg = str(e)

        return res

    def srv_add_urdf(self, req):
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
            res.success = False
            res.error_msg = str(e)

        return res

    def srv_add_controller(self, req):
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
            if req.object_id in self.sensor_publishers:
                self.sensor_publishers[req.object_id].disable(self.sim)
                self.sim.deregister_plugin(self.sensor_publishers[req.object_id])
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
            res.error_msg = str(e)
        return res

    def srv_set_pose(self, req):
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
            res.error_msg = str(e)
        return res

    def srv_save_to_yaml(self, req):
        res = SaveSimulatorResponse()
        res.success = False
        try:
            self.save_to_yaml(req.path, req.use_current_as_initial)
            res.success = True
        except Exception as e:
            res.error_msg = str(e)
        return res

    def srv_save_to_rosparam(self, req):
        res = SaveSimulatorResponse()
        res.success = False
        try:
            self.save_to_rosparam(req.path, req.use_current_as_initial)
            res.success = True
        except Exception as e:
            res.error_msg = str(e)
        return res

    def srv_load_from_yaml(self, req):
        pass

    def srv_load_from_rosparam(self, req):
        pass

    def srv_reset(self, req):
        res = EmptyResponse()
        res.success = False
        try:
            self.sim.reset()
            res.success = True
        except Exception as e:
            res.error_msg = str(e)
        return res

    def srv_set_gravity(self, req):
        res = SetGravityResponse()
        res.success = False
        try:
            self.sim.set_gravity([req.gravity.x, req.gravity.y, req.gravity.z])
            res.success = True
        except Exception as e:
            res.error_msg = str(e)
        return res