import os
import subprocess
import rospy

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.service_simulator_node import ServiceSimulatorNode
from iai_bullet_sim.multibody import MultiBody
from iai_bullet_sim.utils import res_pkg_path
from iai_bullet_sim.ros_plugins import TFPublisher

class FullStatePublishingNode(ServiceSimulatorNode):
    """
    This class implements a node, which automatically publishes the state of the simulation to TF and other topics.
    It does so through the TFPublisher plugin and by automatically starting robot state publisher nodes for all
    multibodies.
    """
    def __init__(self, simulator_class=BasicSimulator):
        """Instantiates the class.

        :param simulatior_class: Simulator implementation to be used.
        :type  simulatior_class: type
        """
        super(FullStatePublishingNode, self).__init__(simulator_class)

        self.state_publishers = {}
        self.publisher_path = '/opt/ros/{}/lib/robot_state_publisher/robot_state_publisher'.format(os.environ['ROS_DISTRO'])
        self.urdf_cache = {}

    def init(self, config_dict=None, mode='direct'):
        """Initializes the node. Starts the subprocesses for the state publishers
        
        :param config_dict: Simulator configuration
        :type  config_dict: dict, NoneType
        :param mode: Simulator mode. gui | direct
        :type  mode: str
        """
        super(FullStatePublishingNode, self).init(config_dict, mode)

        for name in self.sim.bodies.keys():
            self.__start_publisher_for_object(name)                        

        if not self.sim.has_plugin_of_type(TFPublisher):
            self.sim.register_plugin(TFPublisher(self.sim))

    def __start_publisher_for_object(self, name):
        body = self.sim.bodies[name]
        if isinstance(body, MultiBody):
            if len(body.dynamic_joints) > 0:
                if body.urdf_file not in self.urdf_cache:
                    param_path = '/{}/robot_description'.format(name)
                    rospy.set_param(param_path, open(res_pkg_path(body.urdf_file), 'r').read())
                    self.urdf_cache[body.urdf_file] = param_path
                
                param_path = self.urdf_cache[body.urdf_file]

                sp_p = subprocess.Popen([self.publisher_path,
                    '__name:={}_state_publisher'.format(name), 
                    'robot_description:={}'.format(param_path),
                    '_tf_prefix:={}'.format(name),
                    'joint_states:=/{}/joint_states'.format(name)])
                self.state_publishers[name] = sp_p

    def srv_add_urdf(self, req):
        """Override of add URDF service. Starts a state publisher for the newly created object."""
        res = super(FullStatePublishingNode, self).srv_add_urdf(req)
        with self.lock:
            if res.success:
                self.__start_publisher_for_object(res.object_id)
            return res

    def kill(self):
        """Override of shutdown behavior. Kills the subprocesses."""
        super(FullStatePublishingNode, self).kill()
        processes = self.state_publishers.values()
        print('Killing publishers...')
        for x in range(len(processes)):
            processes[x].terminate()
            processes[x].wait()
            print('Killed {}/{}'.format(x + 1, len(processes)))