#!/usr/bin/env python
import rospy
from iai_bullet_sim.realtime_simulator_node import FixedTickSimulator
from iai_bullet_sim.full_state_interactive_node import FullStateInteractiveNode


if __name__ == '__main__':
    rospy.init_node('bullet_realtime_simulator')

    node = FixedTickSimulator(FullStateInteractiveNode)
    node.init_from_rosparam('sim_config', mode='direct')
    node.run()

    while not rospy.is_shutdown():
        pass

    node.kill()