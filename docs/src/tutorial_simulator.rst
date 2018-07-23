Tutorial: Using the Simulator
=============================
In this section we are going to learn how to use the simulator. All of the examples in this section are implemented in the file *scripts/tutorial.py*. They can be executed by passing an example snippet's name as argument during the execution.

Getting Started
---------------

The basic simulator implementation is provided by the module :code:`basic_simulator` in the form of the class :code:`BasicSimulator`.
The following code uses instantiates a simulator, initializes it in gui-mode, adds two objects to it and then updates it at an interval matching its internal update step. After each update, the location of one of the objects is printed.

.. code-block:: python
    :caption: intro

    from time import time
    from iai_bullet_sim.basic_simulator import BasicSimulator

    sim = BasicSimulator()
    sim.init(mode='gui')
    floor   = sim.create_box(extents=[10,10,0.1], mass=0)
    capsule = sim.create_capsule(radius=0.25, height=1,
                                 pos=[0,0,2], rot=[0,1,0,0],
                                 mass=10)

    last_update = time()
    while True:
        if time() - last_update >= sim.time_step:
            sim.update()
            print(str(capsule.pose().position))
            last_update = time()


The code itself is quite straight forward. Important to note however, is that assigning an object a mass of *0* makes this object static.


Using Joints
------------------
After having learned how to use the simulator, let us move on to loading and controlling more complex objects. Specifically, we are going to load a windmill from a URDF, make it's rotor turn and head shake. The current joint states will are printed to the terminal during the simulation.


.. code-block:: python
    :caption: joints

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


The :code:`load_urdf` function is used to instantiate a multi body from a URDF. There are three commands that can be issued to the joints of a loaded object. The joints can be given position, velocity, or effort commands. The multi body offers one method for each command type, e.g :code:`apply_joint_pos_cmds`. These functions are given a dictionary which maps joint names to their respective commands. All commands stay active, until they are replaced by new command. In the example above, the velocity command for the rotor is only given once, but still the rotor keeps turning throughout the demo. The positional command for the head is replaced during each update cycle, so that the head keeps performing a shaking motion. 

The state of a multi body's joints can be accessed using the :code:`joint_state` method. It returns a dictionary, mapping joint names to :code:`JointState` structures, which contain the joint's current position, velocity, exerted effort and reaction forces. The reaction forces will only be calculated if the force torque sensor is enabled for that joint.


Using Sensors
-------------
Aside from their basic state, joints can be set to additionally compute reaction forces. The following example loads a model of a scale from a URDF, spawns a couple of cubes onto the scale's plate and prints out the linear force acting on the plate's joint. 


.. code-block:: python
    :caption: sensor

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


The method :code:`enable_joint_sensor` is used to enable the reaction force calculation for the plate's joint. During the simulation, :code:`get_sensor_states` is used to get a dictionary mapping joint names to their current reaction forces. 


Contact Points
--------------
Lastly, let us take a look at contact queries. Contact queries are used to listen for contact events between objects. The following example uses the scale and cubes again. It prints out all the objects, that the scale's plate is in contact with.


.. code-block:: python
    :caption: contacts

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


Both rigid and multi bodies have a :code:`get_contacts` method, which will return a list of contact points that the object has with other objects. These contact points can be filtered to be only between two objects, or even to be only between two links of two multi bodies. When the filter options are set to :code:`None`, :code:`get_contacts` will return any contact.
Internally the objects use :code:`BasicSimulator.get_contacts`. This method can be used to get a list of all contacts computed during the last physics update.