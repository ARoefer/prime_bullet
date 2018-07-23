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


