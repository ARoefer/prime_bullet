#!/usr/bin/env python
import rospy
from iai_bullet_sim.realtime_simulator_node import FixedTickSimulator


if __name__ == '__main__':
    rospy.init_node('bullet_realtime_simulator')

    node = FixedTickSimulator()
    node.init_from_rosparam('sim_config', mode='direct')
    node.run()

    while not rospy.is_shutdown():
        pass

    node.kill()