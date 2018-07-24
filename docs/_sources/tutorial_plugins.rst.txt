Tutorial: Plugins
=========================

This chapter will provide an introduction to the simulator's plugin system. It will explain the life cycle of a plugin and will end with a small example for a custom plugin and how it can be serialized.

.. _plugin-lc:

Introduction and Life Cycle
---------------------------

Plugins are a way of extending the simulator's functionality, without subclassing it. Plugins are registered with the simulator and automatically tied into the simulator's update cycle. Every plugin can implement a :code:`pre_physics_update` and :code:`post_physics_update` method, which are then called before and after every physics step by the simulator, respectively.

A plugin is assumed to be initialized once it is instantiated. A plugin can implement a :code:`disable` method, at the end of a plugin's life cycle.


.. _plugin-use:

Using a Plugin
--------------
The following example extends the windmill example from :ref:`ex-windmill`. Instead of printing out the joints' states to the terminal however, it instantiates a plugin, which measures the change in joint positions per physics update.


.. code-block:: python
    :caption: use_plugin

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


As you can see, hooking a plugin up to the simulator requires only a single call to :meth:`.BasicSimulator.register_plugin`.


.. _plugin-creation:

Creating a Plugin
-----------------

Let us take a look at the plugin instantiated in the previous section, to understand how plugins are structured.


.. code-block:: python
    :caption: use_plugin

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
        return {'body': simulator.get_body_id(self.body.bId())}

    @classmethod
    def factory(cls, simulator, init_dict):
        return SimplePlugin(simulator.get_body(init_dict['body']))


All plugins are subclasses of :class:`.SimulatorPlugin`. :meth:`.SimulatorPlugin.__init__` requires a human readable name for the implemented plugin.

As already mentioned in :ref:`plugin-lc`, :meth:`.SimulatorPlugin.pre_physics_update` and :meth:`.SimulatorPlugin.post_physics_update` are called before and after the physics step.
In this example they are used to record the pre-physics joint positions and then compare them to the next positions after the step.

Aside from the run-time functionality, a plugin also needs to implement the serialization functions :code:`to_dict` and :code:`factory`, the latter of which is a class-method.
The :code:`to_dict` generates a dictionary which can be used by :code:`factory` in combination with a simulator to instantiate an equivalent plugin in the context of a given simulator.
These functions are necessary to save and load entire simulator configurations.