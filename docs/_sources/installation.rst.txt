The iai_bullet_sim
==================

This ROS package provides an object oriented wrapper around the `bullet physics <http://bulletphysics.org>`_ engine's Python interface. It is supposed to make working with bullet in Python easier and provides a small simulator which is interfaced with the ROS ecosystem.


Installation
------------
This installation will create a new ROS workspace and use wstool to install the simulator. The instructions were tested on a Ubuntu 16.04 with ROS kinetic and cmake 3.5.1.

.. code-block:: bash
    :caption: Creating a workspace

    source /opt/ros/kinetic/setup.bash          # start using ROS kinetic
    mkdir -p ~/bullet_ws/src                    # create directory for workspace
    cd ~/bullet_ws                              # go to workspace directory
    catkin init                                 # init workspace
    cd src                                      # go to source directory of workspace
    wstool init                                 # init rosinstall


.. code-block:: bash
    :caption: Cloning simulator and Bullet using wstool

    wstool merge https://raw.githubusercontent.com/aroefer/iai_bullet_sim/master/rosinstall/catkin.rosinstall
                                                # update rosinstall file
    wstool update                               # pull source repositories
    rosdep install --ignore-src --from-paths .  # install dependencies available through apt
    cd ..                                       # go to workspace directory
    catkin build                                # build packages
    source ~/bullet_ws/devel/setup.bash         # source new overlay


.. code-block:: bash
    :caption: Building Bullet

    cd ~/bullet_ws/src/bullet3
    ./build_cmake_pybullet_double.sh


To use the Python bindings for Bullet, you need to add them to your **PYTHONPATH**. If did all of the steps above, the built library should be located in *~/bullet_ws/bullet3/build_cmake/examples/pybullet*.
