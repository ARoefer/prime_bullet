Overview
========

This package provides an object oriented wrapper around bullet3's Python interface and a simple physics simulator for the ROS ecosystem.

Introduction
------------

The simulator implemented in this package can be split into three main components:

- The simulator
- The objects
- The plugins
  
The simulator connects to the bullet framework. It steps the simulation and is used to instantiate objects. It can save its state to a Python dictionary and also load a state from these structures.

The objects provide a object-level access to bullet's objects. They come in the form of rigid bodies and multi bodies. Rigid bodies are geometrically simple objects which only have one collider. Multi bodies can consist of multiple sub-bodies - so called *links* - which are connected by *joints*. These objects can handily loaded from URDF files. The joints can be actively controlled and there simulated step can be accessed through the multi body interface.

Plugins allow developers to easily tie in additional behavior into the simulator's update cycle, without having to actually subclass the simulator class. These plugins have to provide appropriate serialization and factory functions so the simulator can save and load them from a configuration dictionary.

Using the Simulator
----------------------------------
In this section we are going to learn how to use the simulator. All of the examples in this section are implemented in the file *scripts/tutorial.py*. They can be executed by passing an example snippet's name as argument during the execution.


The basic simulator implementation is provided by the module :code:`basic_simulator` in the form of the class :code:`BasicSimulator`.
The following code uses instantiates a simulator, initializes it in gui-mode, adds two objects to it and then updates it at an interval matching its internal update step. After each update, the location of one of the objects is printed.

.. code::
	:language: python
	:caption: intro

	from time import time
	from iai_bullet_sim.basic_simulator import BasicSimulator

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


The code itself is quite straight forward. Important to note however, is that assigning an object a mass of *0* makes this object static.


