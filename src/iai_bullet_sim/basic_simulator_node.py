import rospy
import sys
import yaml
from time import time

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.utils           import res_pkg_path


class BasicSimulatorNode(object):
    def __init__(self, simulator_class=BasicSimulator):
        self._simulator_class = simulator_class

    def init_from_yaml(self, yaml_path, mode='gui'):
        """Loads a simulator configuration from a YAML file and starts the simulator.

        :param yaml_path: Path of the file as local or global path, or as ROS package URI.
        :type  yaml_path: str
        :param mode: Simulator mode. gui | direct
        :type  mode: str
        """
        self.init(yaml.load(open(res_pkg_path(yaml_path), 'r')), mode)

    def init_from_rosparam(self, rosparam, mode='gui'):
        """Loads a simulator configuration from a ROS parameter and starts the simulator.

        :param rosparam: Name of the parameter
        :type  rosparam: str
        :param mode: Simulator mode. gui | direct
        :type  mode: str
        """
        self.init(rospy.get_param(rosparam, None), mode)

    def init(self, config_dict=None, mode='gui'):
        """Initializes the simulator using a configuration dictionary or a default configuration.

        :param config_dict: Simulator configuration
        :type  config_dict: dict, NoneType
        :param mode: Simulator mode. gui | direct
        :type  mode: str
        """
        cdict = config_dict if config_dict is not None else {'tick_rate': 50, 'gravity': [0,0,-9.81]}
        self.sim = self._simulator_class()
        self.sim.init(mode)
        self.sim.load_simulator(cdict)

    def tick(self, timer_event):
        """
        :type timer_event: rospy.timer.TimerEvent
        """
        self.sim.update()

    def run(self):
        pass

    def pause(self):
        pass

    def is_running(self):
        raise (NotImplemented)

    def reset(self):
        self.sim.reset()

    def stop(self):
        self.sim.stop()

    def kill(self):
        self.stop()
        self.sim.kill()

    def save_to_yaml(self, yaml_path, use_current_state_as_init=False):
        """Saves the simulator's configuration to a YAML file.

        :param yaml_path: Path of the file as local or global path, or as ROS package URI.
        :type  yaml_path: str
        :param use_current_state_as_init: Should the current state, or the initial state be serialized.
        :type  use_current_state_as_init: bool
        """
        out = open(res_pkg_path(yaml_path), 'w')
        out.write(yaml.dump(self.sim.save_simulator(use_current_state_as_init)))
        out.close()

    def save_to_rosparam(self, rosparam, use_current_state_as_init=False):
        """Saves the simulator's configuration to the ROS parameter server.

        :param rosparam: Root parameter of the configuration
        :type  rosparam: str
        :param use_current_state_as_init: Should the current state, or the initial state be serialized.
        :type  use_current_state_as_init: bool
        """
        rospy.set_param(rosparam, self.sim.save_simulator(use_current_state_as_init))

