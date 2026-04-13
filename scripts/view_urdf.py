#!/usr/bin/env python
import sys
from time import time
from math import sin

import prime_bullet as pb


if __name__ == '__main__':
    
    if len(sys.argv) < 2:
        print('Name of urdf to load required. ROS package URIs are supported.')
    else:
        
        sim = pb.Simulator()
        sim.init(mode='gui')
        floor = sim.create_box(extents=[10,10,0.1], mass=0)
        thing = sim.load_urdf(sys.argv[1], useFixedBase=1)
        box   = sim.create_box(extents=[1,1,1],
                               mass=20,
                               pose=pb.Transform.from_xyz(0,0,10))

        last_update = time()
        while True:
            if time() - last_update >= sim.time_step:
                sim.update()
                last_update = time()
