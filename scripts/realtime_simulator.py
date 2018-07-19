#!/usr/bin/env python
import rospy
from iai_bullet_sim.service_simulator_node import ServiceSimulatorNode

class FixedTickSimulator(ServiceSimulatorNode):
    def __init__(self):
        super(FixedTickSimulator, self).__init__()
        self.timer = None

    def pause(self):
        super(FixedTickSimulator, self).pause()

        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None

    def run(self):
        super(FixedTickSimulator, self).run()
        if self.timer is None:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.sim.tick_rate), self.tick)

    def stop(self):
        self.pause()
        super(FixedTickSimulator, self).stop()


if __name__ == '__main__':
    rospy.init_node('bullet_realtime_simulator')

    node = FixedTickSimulator()
    node.init_from_rosparam('sim_config', mode='direct')
    node.run()

    while not rospy.is_shutdown():
        pass

    node.stop()