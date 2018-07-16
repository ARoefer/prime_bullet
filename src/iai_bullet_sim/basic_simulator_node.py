import rospy
import sys
import yaml 
from time import time

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.ros_plugins     import PLUGINS
from iai_bullet_sim.utils           import res_pkg_path


class BasicSimulatorNode(object):
    def __init__(self):
        pass

    def init_from_yaml(self, yaml_path, mode='gui'):
        self.init(yaml.load(open(res_pkg_path(yaml_path), 'r')))

    def init_from_rosparam(self, rosparam, mode='gui'):
        self.init(rospy.get_param(rosparam, None))

    def init(self, config_dict=None, mode='gui'):
        cdict = config_dict if config_dict is not None else {'tick_rate': 50, 'gravity': [0,0,-9.81]}
        self.sim = BasicSimulator()
        self.sim.init(mode)
        self.sim.load_simulator(cdict, PLUGINS)

    def tick(self, timer_event):
        self.sim.update()

    def run(self):
        pass

    def pause(self):
        pass

    def reset(self):
        self.sim.reset()

    def stop(self):
        self.sim.stop()

    def kill(self):
        self.stop()
        self.sim.kill()

    def save_to_yaml(self, yaml_path, use_current_state_as_init=False):
        out = open(res_pkg_path(yaml_path), 'w')
        out.write(yaml.dump(self.sim.save_simulator(use_current_state_as_init)))
        out.close()

    def save_to_rosparam(self, rosparam, use_current_state_as_init=False):
        rospy.set_param(rosparam, self.sim.save_simulator(use_current_state_as_init))

