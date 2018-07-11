#!/usr/bin/env python
import sys
import rospy
from time import time

from iai_bullet_sim.basic_simulator import BasicSimulator

class DemoIntro(object):
	def __init__(self):
		pass

	def run(self):
		sim = BasicSimulator()
		sim.init(mode='gui')
		floor   = sim.create_box(half_extents=[10,10,0.1], mass=0)
		capsule = sim.create_capsule(radius=0.25, height=1, pos=[0,0,2], rot=[0,1,0,0], mass=10)

		last_update = time()
		while True:
			if time() - last_update >= sim.time_step:
				sim.update()
				print(str(capsule.pose().position))
				last_update = time()


demos = {'intro': DemoIntro}

if __name__ == '__main__':
	if len(sys.argv) < 2:
		print('Name of demo to run required. Options are:\n  {}'.format('\n  '.join(demos.keys())))
	else:
		demo = sys.argv[1]

		if demo not in demos:
			print('Unknown demo {}. Options are:\n  {}'.format(demo, '\n  '.join(demos.keys())))
		else:
			d = demos[demo]()
			d.run()