#!/usr/bin/env python
import rospy
from iai_bullet_sim.full_state_interactive_node import FullStateInteractiveNode

class FixedTickSimulator(FullStateInteractiveNode):
    def __init__(self):
        super(FixedTickSimulator, self).__init__()
        self.timer = None
        self.paused = False

    def pause(self):
        super(FixedTickSimulator, self).pause()
        self.paused = True

    def tick(self, timer_event):
        if not self.paused:
            super(FixedTickSimulator, self).tick(timer_event)
        elif self.tf_publisher != None:
            with self.lock:
                self.tf_publisher.post_physics_update(self.sim, 0)

    def run(self):
        super(FixedTickSimulator, self).run()
        if self.timer is None:
            self.timer = rospy.Timer(rospy.Duration(1.0 / self.sim.tick_rate), self.tick)
        self.paused = False

    def stop(self):
        self.pause()
        super(FixedTickSimulator, self).stop()

    def is_running(self):
        return not self.paused
