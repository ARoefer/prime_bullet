import pybullet as pb
import rospy

from interactive_markers.interactive_marker_server import *

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.full_state_node import FullStatePublishingNode
from iai_bullet_sim.multibody import MultiBody
from iai_bullet_sim.srv import *
from iai_bullet_sim.utils import Frame

from geometry_msgs.msg import Pose as PoseMsg
from std_msgs.msg import String as StringMsg

from visualization_msgs.msg import Marker as MarkerMsg
from visualization_msgs.msg import InteractiveMarkerControl as InteractiveMarkerControlMsg
from visualization_msgs.msg import InteractiveMarkerFeedback as InteractiveMarkerFeedbackMsg

zero_pose = PoseMsg()

class FullStateInteractiveNode(FullStatePublishingNode):
    """This node publishes the state of the simulation and makes objects interactable through interactive markers."""

    def __init__(self, server_name='iai_bullet_sim', simulator_class=BasicSimulator):
        """Instantiates the node.

        :param server_name: Name for the marker server.
        :type  server_name: str
        :param simulator_class: Simulator class to use
        :type  simulator_class: type
        """
        super(FullStateInteractiveNode, self).__init__(simulator_class)

        self._selected_object = None
        self.marker_server     = None
        self.server_name       = server_name
        self.services.append(rospy.Service('select_object', ObjectId, self.srv_select_object))
        self.pub_selection = rospy.Publisher('selected_object', StringMsg, queue_size=1, tcp_nodelay=True)
        self.markers = {}
        self.place_mode = False
        self.tf_publisher = None

    def init(self, config_dict=None, mode='direct'):
        """Node initialization behavior.

        :param config_dict: Simulator configuration
        :type  config_dict: dict, NoneType
        :param mode: Simulator mode. gui | direct
        :type  mode: str
        """
        super(FullStateInteractiveNode, self).init(config_dict, mode)

        self.marker_server = InteractiveMarkerServer(self.server_name)

        for name, body in self.sim.bodies.items():
            if isinstance(body, MultiBody):
                self.tf_publisher.add_object(name, body)
                self.add_new_marker(name, body, self.process_body_marker_feedback, False, True)
            else:
                if body.mass > 0.0:
                    self.tf_publisher.add_object(name, body)
                    self.add_new_marker(name, body, self.process_body_marker_feedback, False, True)
                else:
                    self.add_new_marker(name, body, self.process_static_body_marker_feedback, False, True, 'map', body.pose())
        self.marker_server.applyChanges()

    def on_obj_deleted(self, simulator, Id, obj):
        super(FullStateInteractiveNode, self).on_obj_deleted(simulator, Id, obj)

        if Id in self.markers:
            if Id == self._selected_object:
                self.select_marker(None)
            self.marker_server.erase(Id)
            self.marker_server.applyChanges()
            del self.markers[Id]


    def add_new_marker(self, name, body, callback, selected=True, delay_update=False, frame=None, pose=None):
        """Adds a new interactive marker to the node.

        :param name: Name of the marker
        :type  name: str
        :param body: Body controlled by the marker
        :type  body: RigidBody, MultiBody
        :param callback: Callback function for the marker
        :type  callback: function
        :param selected: Should the marker be selected upon creation
        :type  selected: bool
        :param delay_update: Delay the update to the server
        :type  delay_update: bool
        :param frame: Name of reference frame
        :type  frame: NoneType, str
        :param pose: Initial pose of the marker
        :type  pose: NoneType, Frame
        """
        intMarker = InteractiveMarker()
        intMarker.name = name
        intMarker.header.frame_id = frame if frame is not None else name
        intMarker.header.stamp = rospy.Time(0)
        intMarker.scale = 1.0
        if pose is not None:
            intMarker.pose.position.x = pose.position[0]
            intMarker.pose.position.y = pose.position[1]
            intMarker.pose.position.z = pose.position[2]
            intMarker.pose.orientation.x = pose.quaternion[0]
            intMarker.pose.orientation.y = pose.quaternion[1]
            intMarker.pose.orientation.z = pose.quaternion[2]
            intMarker.pose.orientation.w = pose.quaternion[3]

        control = InteractiveMarkerControlMsg()
        control.interaction_mode = InteractiveMarkerControlMsg.MOVE_3D
        control.always_visible = True
        control.orientation.w = 1.0
        control.name = 'visual'
        control.description = name
        intMarker.controls.append(control)
        make6DOFGimbal(intMarker)
        self.create_visual_marker(name, body, intMarker, control)

        activateMarker(intMarker, selected)
        self.marker_server.insert(intMarker, callback)
        self.markers[name] = (intMarker, callback)

        body.register_deletion_cb(self.on_obj_deleted)

        if not delay_update:
            self.marker_server.applyChanges()

    def create_visual_marker(self, name, data, intMarker, control):
        """Creates a visual marker from a body.

        :param name: Name of the marker
        :type  name: str
        :param data: Body to represent
        :type  data: RigidBody, MultiBody
        :param intMarker: Marker this body is contained in
        :type  intMarker: InteractiveMarker
        :param control: Control the visual should be attached to
        :type  control: InteractiveMarkerControlMsg
        """
        if isinstance(data, MultiBody):
            intMarker.header.frame_id = '{}/{}'.format(name, data.base_link)
        else:
            control.markers.extend(rigid_body_to_markers(data))
            max_dim = max(control.markers[0].scale.x, control.markers[0].scale.y, control.markers[0].scale.z)
            intMarker.scale = max_dim + 0.1

    def srv_add_urdf(self, req):
        """Override of URDF-adding service. Automatically adds an interactive marker for the object."""
        res = super(FullStateInteractiveNode, self).srv_add_urdf(req)
        if res.success:
            self.add_new_marker(res.object_id, self.sim.bodies[res.object_id], self.process_body_marker_feedback)
        return res

    def srv_add_rigid_body(self, req):
        """Override of rigid body adding service. Automatically adds an interactive marker for the object."""
        res = super(FullStateInteractiveNode, self).srv_add_rigid_body(req)
        if res.success:
            self.add_new_marker(res.object_id, self.sim.bodies[res.object_id], self.process_body_marker_feedback)
        return res

    def srv_select_object(self, req):
        """Service changing the currently selected object."""
        res = ObjectIdResponse()
        if req.object_id in self.sim.bodies:
            self.select_marker(req.object_id)
            res.success = True
        elif len(req.object_id) == 0:
            self.select_marker(None)
            res.success = True
        else:
            res.error_msg = 'Unknown object {}.'.format(req.object_id)
        return res


    def select_marker(self, marker_id):
        """Selects the marker matching the given Id.

        :param marker_id: Marker to select
        :type  marker_id: str
        """
        if self._selected_object != marker_id:
            if self._selected_object in self.markers:
                cm, cb = self.markers[self._selected_object]
                activateMarker(cm, False)
                self.marker_server.insert(cm, cb)
            self._selected_object = marker_id
            if self._selected_object in self.markers:
                cm, cb = self.markers[self._selected_object]
                activateMarker(cm, True)
                self.marker_server.insert(cm, cb)
            self.marker_server.applyChanges()
            msg = StringMsg()
            msg.data = self._selected_object if self._selected_object != None else ''
            self.pub_selection.publish(msg)


    def process_body_marker_feedback(self, feedback):
        """Processes the feedback sent by markers corresponding to dynamic objects."""
        if feedback.event_type == InteractiveMarkerFeedbackMsg.MOUSE_DOWN and feedback.marker_name != self._selected_object:
            self.select_marker(feedback.marker_name)

        if feedback.event_type == InteractiveMarkerFeedbackMsg.POSE_UPDATE:
            if feedback.marker_name != self._selected_object:
                self.select_marker(feedback.marker_name)
            if self.is_running():
                self.pause()
                self.place_mode = True

        if feedback.event_type == InteractiveMarkerFeedbackMsg.MOUSE_UP:
            body = self.sim.bodies[self._selected_object]
            obj_pose = body.pose()
            rel_frame = pose_msg_to_frame(feedback.pose)

            new_pos, new_rot = pb.multiplyTransforms(list(obj_pose.position), list(obj_pose.quaternion), list(rel_frame.position), list(rel_frame.quaternion))
            body.set_pose(Frame(new_pos, new_rot), True)
            intMarker, cb = self.markers[self._selected_object]
            intMarker.pose = zero_pose
            self.marker_server.insert(intMarker, cb)
            self.marker_server.applyChanges()

            if self.place_mode:
                self.place_mode = False
                self.run()

    def process_static_body_marker_feedback(self, feedback):
        """Processes feedback sent by markers corresponding to static objects."""
        if feedback.event_type == InteractiveMarkerFeedbackMsg.MOUSE_DOWN and feedback.marker_name != self._selected_object:
            self.select_marker(feedback.marker_name)

        if feedback.event_type == InteractiveMarkerFeedbackMsg.POSE_UPDATE:
            if feedback.marker_name != self._selected_object:
                self.select_marker(feedback.marker_name)
            if self.is_running():
                self.pause()
                self.place_mode = True

        if feedback.event_type == InteractiveMarkerFeedbackMsg.MOUSE_UP:
            body = self.sim.bodies[self._selected_object]
            body.set_pose(pose_msg_to_frame(feedback.pose), True)
            intMarker, cb = self.markers[self._selected_object]
            intMarker.pose = feedback.pose

            if self.place_mode:
                self.place_mode = False
                self.run()

def pose_msg_to_frame(msg):
    """Converts a geometry_msgs.msg.Pose to a Frame."""
    return Frame([msg.position.x, msg.position.y, msg.position.z],
                 [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w ])


def rigid_body_to_markers(body):
    """Helper function generating visual markers for physical bodies."""
    central_marker = MarkerMsg()
    out = [central_marker]
    central_marker.pose.orientation.w = 1
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
        if body.type == 'capsule':
            central_marker.scale.z = body.height - 2 * body.radius
            upper_sphere = MarkerMsg()
            upper_sphere.type  = MarkerMsg.SPHERE
            upper_sphere.color = central_marker.color
            upper_sphere.scale.x = body.radius
            upper_sphere.pose.position.z = body.height * 0.5 - body.radius
            upper_sphere.pose.orientation.w = 1
            lower_sphere = MarkerMsg()
            lower_sphere.type  = MarkerMsg.SPHERE
            lower_sphere.color = central_marker.color
            lower_sphere.scale.x = body.radius
            lower_sphere.pose.position.z = -body.height * 0.5 + body.radius
            lower_sphere.pose.orientation.w = 1
            out.append(lower_sphere)
            out.append(upper_sphere)
        else:
            central_marker.scale.z = body.height

    return out


def make6DOFGimbal(intMarker):
    """Creates 6DOF gimbal marker controls."""
    pitch = InteractiveMarkerControlMsg()
    pitch.orientation.x = 1
    pitch.orientation.y = 0
    pitch.orientation.z = 0
    pitch.orientation.w = 1
    pitch.interaction_mode = InteractiveMarkerControlMsg.ROTATE_AXIS
    pitch.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(pitch)

    yaw = InteractiveMarkerControlMsg()
    yaw.orientation.x = 0
    yaw.orientation.y = 1
    yaw.orientation.z = 0
    yaw.orientation.w = 1
    yaw.interaction_mode = InteractiveMarkerControlMsg.ROTATE_AXIS
    yaw.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(yaw)

    roll = InteractiveMarkerControlMsg()
    roll.orientation.x = 0
    roll.orientation.y = 0
    roll.orientation.z = 1
    roll.orientation.w = 1
    roll.interaction_mode = InteractiveMarkerControlMsg.ROTATE_AXIS
    roll.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(roll)

    transX = InteractiveMarkerControlMsg()
    transX.orientation.x = 1
    transX.orientation.y = 0
    transX.orientation.z = 0
    transX.orientation.w = 1
    transX.interaction_mode = InteractiveMarkerControlMsg.MOVE_AXIS
    transX.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(transX)

    transY = InteractiveMarkerControlMsg()
    transY.orientation.x = 0
    transY.orientation.y = 1
    transY.orientation.z = 0
    transY.orientation.w = 1
    transY.interaction_mode = InteractiveMarkerControlMsg.MOVE_AXIS
    transY.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(transY)

    transZ = InteractiveMarkerControlMsg()
    transZ.orientation.x = 0
    transZ.orientation.y = 0
    transZ.orientation.z = 1
    transZ.orientation.w = 1
    transZ.interaction_mode = InteractiveMarkerControlMsg.MOVE_AXIS
    transZ.orientation_mode = InteractiveMarkerControlMsg.FIXED
    intMarker.controls.append(transZ)


def activateMarker(intMarker, active):
    """Updates a marker to the selected or non-selected visuals."""
    if active:
        #intMarker.controls[0].interaction_mode = InteractiveMarkerControlMsg.MOVE_3D
        for control in intMarker.controls[1:4]:
            control.interaction_mode = InteractiveMarkerControlMsg.ROTATE_AXIS
        for control in intMarker.controls[4:]:
            control.interaction_mode = InteractiveMarkerControlMsg.MOVE_AXIS
    else:
        #intMarker.controls[0].interaction_mode = InteractiveMarkerControlMsg.BUTTON
        for control in intMarker.controls[1:]:
            control.interaction_mode = InteractiveMarkerControlMsg.NONE
