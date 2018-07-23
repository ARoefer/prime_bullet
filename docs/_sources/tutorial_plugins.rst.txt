Tutorial: Plugins
=========================

This chapter will provide an introduction to the simulator's plugin system. It will explain the life cycle of a plugin and will end with a small example for a custom plugin and how it can be serialized.


Introduction and Life Cycle
---------------------------

Plugins are a way of extending the simulator's functionality, without subclassing it. Plugins are registered with the simulator and automatically tied into the simulator's update cycle. Every plugin can implement a :code:`pre_physics_update` and :code:`post_physics_update` method, which are then called before and after every physics step by the simulator, respectively.

A plugin is assumed to be initialized once it is instantiated. A plugin can implement a :code:`disable` method, at the end of a plugin's life cycle.


Using a Plugin
--------------
In this example