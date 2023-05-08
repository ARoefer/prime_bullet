#!/usr/bin/env python
import sys
from time import time
from math import sin

from iai_bullet_sim.basic_simulator import BasicSimulator, SimulatorPlugin

from iai_bullet_sim import Transform, \
                           Point3,    \
                           Vector3

class DemoIntro(object):
    def __init__(self):
        pass

    def run(self):
        sim = BasicSimulator()
        sim.init(mode='gui')
        floor   = sim.create_box(extents=[10,10,0.1], mass=0)
        capsule = sim.create_capsule(radius=0.25, height=1, pos=[0,0,2], rot=[0,1,0,0], mass=10)

        last_update = time()
        while True:
            if time() - last_update >= sim.time_step:
                sim.update()
                print(str(capsule.pose().position))
                last_update = time()


class DemoJoints(object):
    def __init__(self):
        pass

    def run(self):
        sim = BasicSimulator()
        sim.init(mode='gui')
        floor    = sim.create_box(extents=[10,10,0.1], mass=0)
        windmill = sim.load_urdf('package://iai_bullet_sim/urdf/windmill.urdf', useFixedBase=1)

        windmill.apply_joint_vel_cmds({'wings_rotor': -2})

        last_update = time()
        while True:
            if time() - last_update >= sim.time_step:
                windmill.apply_joint_pos_cmds({'head_pan': sin(time())})
                sim.update()
                print('Joint state:\n  {}'.format('\n  '.join(['{:>12}: {}'.format(j, s.position) for j, s in windmill.joint_state().items()])))
                last_update = time()


class DemoSensor(object):
    def __init__(self):
        pass

    def run(self):
        sim = BasicSimulator()
        sim.init(mode='gui')
        floor = sim.create_box(extents=[10,10,0.1], mass=0)
        scale = sim.load_urdf('package://iai_bullet_sim/urdf/scale.urdf', pos=[0,0,0.1], useFixedBase=1)

        for x in range(5):
            sim.create_box(extents=[0.2,0.2,0.2], pos=[0,0,2 + x*0.5], mass=20)

        scale.apply_joint_pos_cmds({'plate_lift': 0.2})
        scale.enable_joint_sensor('plate_lift')

        last_update = time()
        while True:
            if time() - last_update >= sim.time_step:
                sim.update()
                print('Joint state:\n  {}'.format('\n  '.join(['{:>12}: {}'.format(j, str(s.f)) for j, s in scale.get_sensor_states().items()])))
                last_update = time()


class DemoContacts(object):
    def __init__(self):
        pass

    def run(self):
        sim = BasicSimulator(30)
        sim.init(mode='gui')
        floor = sim.create_box(extents=[10,10,0.1], mass=0)
        scale = sim.load_urdf('package://iai_bullet_sim/urdf/scale.urdf', 
                              pose=Transform.from_xyz(0,0,0.1), useFixedBase=1)

        for x in range(5):
            sim.create_box(extents=[0.2,0.2,0.2], pose=Transform.from_xyz(0,0,2 + x*0.5), mass=20)

        scale.apply_joint_pos_cmds({'plate_lift': 0.2})
        sensor = scale.get_ft_sensor('plate_lift')

        plate_link = scale.links['plate']

        last_update = time()
        while True:
            if time() - last_update >= sim.time_step:
                sim.update()
                contacts = plate_link.get_contacts()
                print('Contacts with plate:\n  {}'.format('\n  '.join([str(c.bodyB.bId) for c in contacts])))
                last_update = time()


class SimplePlugin(SimulatorPlugin):
    def __init__(self, multibody):
        super(SimplePlugin, self).__init__('Simple Plugin')
        self.body = multibody

    def pre_physics_update(self, simulator, deltaT):
        self.pre_physics_jp = {j: s.position for j, s in self.body.joint_state().items()}

    def post_physics_update(self, simulator, deltaT):
        jp_delta = {j: s.position - self.pre_physics_jp[j] for j, s in self.body.joint_state().items()}
        print('\n'.join(['{:>20} moved {: 2.6f} rad'.format(j, d) for j, d in jp_delta.items()]))

    def to_dict(self, simulator):
        return {'body': simulator.get_body_id(self.body.bId)}

    @classmethod
    def factory(cls, simulator, init_dict):
        return SimplePlugin(simulator.get_body(init_dict['body']))


class DemoPluginUsage(object):
    def __init__(self):
        pass

    def run(self):
        sim = BasicSimulator()
        sim.init(mode='gui')
        floor    = sim.create_box(extents=[10,10,0.1], mass=0)
        windmill = sim.load_urdf('package://iai_bullet_sim/urdf/windmill.urdf', useFixedBase=1)

        plugin = SimplePlugin(windmill)
        sim.register_plugin(plugin)

        windmill.apply_joint_vel_cmds({'wings_rotor': -2})

        last_update = time()
        while True:
            if time() - last_update >= sim.time_step:
                windmill.apply_joint_pos_cmds({'head_pan': sin(time())})
                sim.update()
                last_update = time()


class DemoMesh(object):
    """docstring for DemoMesh"""
    def __init__(self):
        pass

    def run(self):
        sim = BasicSimulator()
        sim.init(mode='gui')
        floor    = sim.create_box(extents=[10,10,0.1], mass=0)
        suzanne  = sim.load_mesh('package://iai_bullet_sim/meshes/suzanne.dae', pos=[0,0,2])

        last_update = time()
        while True:
            if time() - last_update >= sim.time_step:
                sim.update()
                last_update = time() 


demos = {'intro': DemoIntro,
         'joints': DemoJoints,
         'sensor': DemoSensor,
         'contacts': DemoContacts,
         'use_plugin': DemoPluginUsage,
         'load_mesh' : DemoMesh} # Contacts, closest points,

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Name of demo to run required. Options are:\n  {}'.format('\n  '.join(sorted(demos.keys()))))
    else:
        demo = sys.argv[1]

        if demo not in demos:
            print('Unknown demo {}. Options are:\n  {}'.format(demo, '\n  '.join(sorted(demos.keys()))))
        else:
            d = demos[demo]()
            d.run()

        