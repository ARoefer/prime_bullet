#!/usr/bin/env python
import sys
import rospy
from time import time
from math import sin

from iai_bullet_sim.basic_simulator import BasicSimulator, SimulatorPlugin


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
                contacts = scale.get_contacts(own_link='plate')
                print('Contacts with plate:\n  {}'.format('\n  '.join([sim.get_body_id(c.bodyB.bId()) for c in contacts])))
                last_update = time()


class SimplePlugin(SimulatorPlugin):
    def __init__(self, multibody):
        super(SimplePlugin, self).__init__('Simple Plugin')
        self.body = multibody

    def pre_physics_update(self, simulator, deltaT):
        self.pre_physics_js = self.body.joint_state()

    def post_physics_update(self, simulator, deltaT):
        """Implements post physics step behavior.

        :type simulator: BasicSimulator
        :type deltaT: float
        """
        pass

    def __str__(self):
        return self.__name

    def to_dict(self, simulator):
        """Serializes this plugin to a dictionary.

        :type simulator: BasicSimulator
        :rtype: dict
        """


class DemoPluginUsage(object):
    def __init__(self):
        pass

    def run(self):
        rospy.init_node('plugin_example')
        sim = BasicSimulator()
        sim.init(mode='gui')
        floor    = sim.create_box(extents=[10,10,0.1], mass=0)
        windmill = sim.load_urdf('package://iai_bullet_sim/urdf/windmill.urdf', useFixedBase=1)

        plugin = JSPublisher(windmill, 'windmill')
        sim.register_plugin(plugin)

        windmill.apply_joint_vel_cmds({'wings_rotor': -2})

        last_update = time()
        while True:
            if time() - last_update >= sim.time_step:
                windmill.apply_joint_pos_cmds({'head_pan': sin(time())})
                sim.update()
                last_update = time()


demos = {'intro': DemoIntro,
         'joints': DemoJoints,
         'sensor': DemoSensor,
         'contacts': DemoContacts,
         'use_plugin': DemoPluginUsage} # Contacts, closest points,

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