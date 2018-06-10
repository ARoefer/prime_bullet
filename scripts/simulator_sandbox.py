#!/usr/bin/env python

from iai_bullet_sim.basic_simulator import BasicSimulator

if __name__ == '__main__':
	sim = BasicSimulator(50)
	sim.init(mode='gui')

	plane = sim.load_urdf('package://iai_bullet_sim/urdf/plane.urdf', useFixedBase=1)

	ur5 = sim.load_urdf('package://iai_table_robot_description/robots/ur5_table.urdf', useFixedBase=1)


	while True:
		sim.update()