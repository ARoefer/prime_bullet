[![Total alerts](https://img.shields.io/lgtm/alerts/g/ARoefer/iai_bullet_sim.svg?style=flat-square&logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/ARoefer/iai_bullet_sim/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/ARoefer/iai_bullet_sim.svg?style=flat-square&logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/ARoefer/iai_bullet_sim/context:python)
[![Language grade: JavaScript](https://img.shields.io/lgtm/grade/javascript/g/ARoefer/iai_bullet_sim.svg?style=flat-square&logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/ARoefer/iai_bullet_sim/context:javascript)

# Prime Bullet

Prime Bullet is an object-oriented wrapper for PyBullet which tries to offer a game-engine like interaction with the physics simulator. Aside from the structuring of data, it also include a small SE3/SO3 math library compatible with bullet and numpy, implementations of cameras and laser scanner, as well as simple controllers for robot. Its primary audience are roboticists who are researching manipulation -- both mobile and static.


## Installation

Installing *prime-bullet* is easy. Simply clone the repository, go to its root and run `pip install -e .`:

```bash
git clone https://github.com/ARoefer/iai_bullet_sim

cd iai_bullet_sim

pip install -e .
```

For the usage with ROS *noetic*, we recommend [rosvenv](https://github.com/ARoefer/rosvenv) as a companion tool.


## Getting Started

A brief overview to get you into the features of *prime bullet*.

### The Simulator

Prime Bullet uses a simulator instance as interface to bullet. In case you are familiar with the inner workings of bullet: This instance maintains the connection to the bullet server.

```python
import prime_bullet as pb

# Simulator with 50Hz rate, standard gravity, and no EGL rendering
sim = pb.Simulator()

# Establishes connection with Bullet in 'gui' mode. There is also headless, aka 'direct' mode
sim.init('gui')

# Second simulator with 100Hz rate.
sim2 = pb.Simulator(100)
# New connection to bullet. This simulator is completely separate from the first
sim2.init('direct')
```

The typical life-cycle for the simulator looks like this:

```python
import prime_bullet as pb

sim = pb.Simulator()
sim.init('gui')

# Create a couple objects
# ....

# Simulate
while not some_stopping_condition:
    # Preforms the actual physics step
    sim.update()
    # Maybe do something with the result

    # Maybe reset the sim after a while, if you are doing RL or something similar:
    if episode_ended:
        sim.reset()

# Actually only needed when using plugins -> We'll talk about them later.
sim.stop()
# Closes the connection to bullet
sim.kill()
```

### Objects

Creation of objects is handled through the simulator, as objects are always associated with exactly one simulation. The following object types can be created:

```python
import prime_bullet as pb

sim = pb.Simulator()
sim.init('gui')

# Create a couple objects
# A red box with 20cm edges at (0, 1, 0.2)
box      = sim.create_box([0.2] * 3, pb.Transform.from_xyz(0, 1, 0.2))

# A blue sphere with 30cm radius at (0, 2, 0.6)
sphere   = sim.create_sphere(0.3, pb.Transform.from_xyz(0, 2, 0.6))

# A 1m tall green cylinder with a radius of 20cm at (0, 3, 1)
cylinder = sim.create_cylinder(0.2, 1, pb.Transform.from_xyz(0, 3, 1))

# A monkey mesh at (0, 4, 1)
mesh     = sim.create_mesh('package://iai_bullet_sim/meshes/suzanne.dae', pose=pb.Transform.from_xyz(0, 4, 1))

# A URDF of a windmill at (0, 5, 0)
windmill = sim.load_urdf('package://iai_bullet_sim/src/iai_bullet_sim/data/urdf/windmill.urdf', pb.Transform.from_xyz(0, 5, 0))

# Simulate
while not some_stopping_condition:
    # Preforms the actual physics step
    sim.update()

    # Print the pose of the box after every simulation step
    print(f'Box pose: {box.pose}')

```

There are two types of objects in prime bullet: Rigid bodies and articulated objects. The latter are an extension of the former, which we will talk about in more detail in a second.
Rigid bodies have a number of convenience functions, but primarily it is important to point out the following:

```python
import prime_bullet as pb

sim = pb.Simulator()
sim.init('gui')

box = sim.create_box([0.2] * 3, pb.Transform.from_xyz(0, 1, 0.2))

# Current pose
a = box.pose  
# Set a new pose
a.pose = pb.Transform.from_xyz(1, 2, 3)

# Initial pose
a = box.initial_pose
# Set new initial pose, does not move the object
box.initial_pose = pb.Transform.from_xyz(1, 2, 3)

# Object is reset to initial pose
box.reset()

# Get the current linear velocity
vl = box.linear_velocity

# Get the current angular velocity
va = box.linear_velocity

# Get the current AABB of the object
aabb = box.aabb

```

Articulated objects, such as robots, are more complicated, as they contain joints and their states. Notable functions are:

```python
import prime_bullet as pb

sim = pb.Simulator()
sim.init('gui')

mill = sim.load_urdf('package://iai_bullet_sim/src/iai_bullet_sim/data/urdf/windmill.urdf', pb.Transform.from_xyz(0, 5, 0))

# Links
l = mill.i_links[1]    # Get a link by index
l = mill.links['head'] # Get a link by its name
l.name  # Name of the link
l.pose  # World pose of the link
l.state # Complete state of the link
l.state.linear_velocity   # Current linear velocity
l.state.angular_velocity  # Current angular velocity
l.aabb  # Current AABB of the link
l.jacobian(q, q_dot, q_ddot, point) # Calculate Jacobian for point in frame of the link, given the current q, qd, and desired qdd
l.ik(world_pose, max_iterations)    # Calculate IK for a desired object position or pose. Returns q as np.array

# Joints
# List of all joint names, in order
joints  = mill.joint_names
# List of names of dynamic joints, aka non-static ones, in order
djoints = mill.dynamic_joint_names

joint   = mill.joints['some_joint'] # Get a joint by its name
joint   = mill.i_joints[2]          # Get a joint by its index
# Some of the attributes of a joint (there are many more)
joint.limits # The joint's limits
joint.f_max  # The joint's maximum torque/effort
joint.qd_max # The joint's maximum velocity
joint.link   # The child link of this joint
joint.parent # The parent link of this joint
joint.is_dynamic # Indicates whether the joint is fixed or dynamic

# Current dynamic joint data as np.array, same order as djoints
q  = mill.q     # Positions
qd = mill.q_dot # Velocities
qf = mill.q_f   # Torques

# Current structured dynamic joint information as dictionary
js = mill.joint_state

# Set current joint positions from a dictionary
mill.set_joint_positions(js, override_initial=False)

# Set current dynamic joint positions from a np.array and override joint pose the object is reset to
mill.set_joint_positions(q, override_initial=True)

# Resets both root pose and joint pose
mill.reset()

# Controlling the joints
# Setting a joint position goal as dict or np.array with max forces (np.array)
mill.apply_joint_pos_cmds(cmd, max_force)
# Setting a joint velocity goal as dict or np.array with max forces (np.array)
mill.apply_joint_vel_cmds(cmd, max_force)
# Setting a joint torque goal as dict or np.array
mill.apply_joint_torque_cmds(cmd)

# Returns Force-Torque sensor for the given joint. Joint can also be fixed
ft_sensor = mill.get_ft_sensor('some_joint')
wrench = ft_sensor.get()  # Current wrench measurement
```

### Search Paths
As you could see before, prime bullet is able to use *package* paths. In case you are working with ROS, prime bullet will automatically add your ROS package path to the paths it searches. In case you are using prime bullet without ROS, you can always add paths manually with `add_search_path()`:

```python
import prime_bullet as pb

pb.add_search_path('my_awesome_library_root')
pb.add_search_path('some_other_place/my_package')

# Will be resolved successfully if bla is a directory contained in one of the dirs in the search path
path_a = pb.res_package_path('package://bla/some_file.txt')

# Will be resolved because 'my_package' is part of the search paths.
path_b = pb.res_package_path('package://my_package/some_file.txt')
```

As pybullet is limited to a single search path, prime bullet re-writes all loaded URDF files to use global paths. It is important, that all the packages mentioned in the URDF can be found over the search paths.

### Spatial Transformations
Throughout the examples, we have encountered `pb.Transform` multiple times. This class is part of prime bullet's micro spatial transformation library, which we will introduce you to briefly.

The module implements only 4 datatypes: `Vector3`, `Point3`, `Quaternion`, and `Transform`. The first three are extensions of Python's `tuple` type and are thus compatible with both `pybullet` and `numpy` without any need for conversions. However, when combined with one another they do follow the rules of SE3 algebra. Let us look at a couple examples:

```python
import numpy as np
from prime_bullet import Vector3, Point3, Quaternion, Transform

p1 = Point3(2, 0, 0)
v1 = Vector3(0, 1, 0)

q1 = Quaternion.from_axis_angle(Vector3.unit_z(), np.deg2rad(90))
q2 = Quaternion.from_rpy(np.deg2rad(90), 0, 0)
t1 = Transform(q, Point3(0, 0, 3))

# Points and Vectors
p2 = p1 + v1  # Point +/- Vector -> Point
# This is the only difference in combining points and vectors. 
# As points are an extension of vectors, they behave the same in all other cases
v2 = p1 - p2  # Point - Point -> Vector
p3 = p1 + p2  # Technically nonsensical, but we are merciful and say -> Point

v1 * v2  # Component-wise multiplication
v1 / v2  # Component-wise division

v1.norm()     # L2 norm of vector/point
v1.dot(v2)    # Dot-product of vectors/points
v1.cross(v2)  # Cross-product of vectors/points
v1.numpy()    # Representation as numpy array

v1.x,  v1.y,  v1.z  # Semantic accessing of elements
v1[0], v1[1], v1[2] # Indexed accessing of elements

# Quaternions
q3 = q1.dot(q2)       # Combined rotation of roll and yaw
a  = q1.angle()       # Angle of q1
ad = q1.angle(q2)     # Angle between q1 and q2
q1.inv()              # Inversion of q1
qd = q1.inv().dot(q2) # Delta rotation from q1 to q2
q1.matrix()           # 3x3 Rotation matrix of q1
q1.numpy()            # Quaternion as (4,) np.array
q1.lerp(q2, 0.5)      # 50% interpolation from q1 to q2

p4 = q1.dot(p3)       # Rotate point p3 around origin by q1
v3 = q1.dot(v2)       # Rotate vector v2 around origin by q1

t1.position   # t1's translation -> point
t1.quaternion # t1's orientation -> quaternion
t1.inv()      # Inversion of transform t1
t2 = Transform.from_xyz_rpy(2, 3, 4, 
                            0, np.deg2rad(45), 0)
t1.matrix()           # 4x4 Homogenous transformation matrix
t3 = t1.dot(t2)       # t2 transformed by t1
p5 = t1.dot(p1)       # p1 rotated 90 deg around t1's z-axis and translated by (0, 0, 3)
v4 = t1.dot(v1)       # v1 rotated 90 deg around z-axis
q4 = t1.dot(q3)       # q3 rotated by the rotation of t1
td = t1.relative(t2)  # Relative transform between t1 and t2
t4 = t1.lerp(t2, 0.5) # 50% transformation between t1 and t2
```

There are more constructors for all these data types and more operators. It is worthwhile to check the class reference for these types/the `geometry.py` file.

### Frames
Prime bullet adds the concept of frames, which allow users to build transformation hierarchies. A frame always has a `local_pose` and a `pose`, which describe their transformation either relative to their parent, or relative to the world. All bodies and all links implement the frame concept, however here both of these attributes are always identical, as pybullet does not support the concept of frames. Nonetheless, frames become useful when we want to attach "virtual" object, such as cameras or other sensors, to the bodies managed by the physics simulation. As an example, let us take a look at the `PerspectiveCamera` class.

```python
import prime_bullet as pb

link = some_body.links['link']

# TODO: What's the uint of FOV?
cam  = pb.PerspectiveCamera(sim, (200, 200), 70, 0.1, 10.0, pb.Transform.from_xyz(0.1, 0, 0), parent=link)

# Behind the scenes, the current camera pose is determined before an image is generated.
# Thus the camera is now rigidly attached to 'link'
rgb  = cam.rgb()
```

### Additional Sensors
We don't have the time to go into details, but we would like to point out that prime bullet provides additional sensors. As we have seen in the previous section, there is a `PerspectiveCamera`, but there is also a laser scanner:


```python
import prime_bullet as pb

# Camera rendering 200x200 pixel images at 70deg FOV in a volume from 10cm-10m
cam  = pb.PerspectiveCamera(sim, (200, 200), 70, 0.1, 10.0)
rgb        = cam.rgb()   # RGB image
depth      = cam.depth() # Single channel depth image
rgb, depth = cam.rgbd()  # Both modalities at once
seg        = cam.segmentation()  # Pybullet's segmentation mask

# Laser scanner with 180deg coverage, 1000 rays, measuring from 50cm to 10m
lscanner  = pb.LaserScanner(sim, np.deg2rad(-90), np.deg2rad(90), 1000, 0.5, 10.0, pb.Transform.identity())
distances = lscanner.render() # Distances measured by the rays
```

For more details, please check the class references. It would also be nice to have a 3d lidar and an orthonormal camera (*hint, hint*).

### Controllers
In addition to new sensor modalities, prime bullet also provides a couple controllers to get you started with (Cartesian) robotic control. The following is not going into detail on their usage, but rather serves as an overview.

```python
import prime_bullet as pb

# Control for joint positions. Holds current position by default
jc = pb.JointPositionControl(robot)
jc.reset()  # Assume current robot pose as target
jc.delta    # joint space delta
jc.goal     # Current q target
jc.act(q)   # Set a new q target

# Control for Cartesian pose of link. Assume current pose by default
cc = pb.CartesianController(robot, link)  
cc.reset()  # Assume current link pose as target
cc.delta    # 2d array of translational and angular error
cc.goal     # Current x target
cc.act(x)   # Set a new x target

# Control for Cartesian pose of link with relative actions. Assume current pose by default
cc = pb.CartesianRelativeController(robot, link)  
cc.reset()  # Assume current link pose as target
cc.delta    # 2d array of translational and angular error
cc.goal     # Current x target
cc.act(xd)  # Apply xd transform to current target

# Same as CartesianRelativeController, but without orientation goal
pb.CartesianRelativePointController

# Same as CartesianRelativePointController, but holding a fixed orientation
pb.CartesianRelativePointCOrientationController

# Same as CartesianRelativeController, but a virtual point is moved, unrelated to the link's pose
pb.CartesianRelativeVirtualPointController

# Same as CartesianRelativeVirtualPointController, but while holding a fixed orientation
pb.CartesianRelativeVPointCOrientationController
```

### Other Things
There are three more things in this library to mention:
 
 1. The `pb.ColorRGBA` type, which behaves like the other types, but represents colors and defines a few constant ones.
 2. The `pb.AABB` type which represents axis aligned bounding boxes and can be used to check if points are within such a box.
 3. The `pb.DebugVisualizer` which can be obtained from a `Simulator` instance which is currently in `gui` mode. This class can be used to control pybullet's visualizer.


### Plugins
TODO. But generally, they allow users to attach additional functionality to the `update()` of the simulator. Check the `SimulatorPlugin` class for details.

## Usage with `gym.Env`

Briefly, let us remark on the usage of prime bullet with the OpenAI `gym` API. Since the `Simulator` class seems to have a functional overlap with a `gym.Env`, it might be tempting to create a custom environment by deriving a class from both `Simulator` and `gym.Env`. We advise against this, as it will create a messy architecture in which different semantic goals and timesteps collide. Instead, we propose the following as a rough skeleton for using prime bullet with gym environments:

```python
import gym
import prime_bullet as pb

class MyEnv(gym.Env):
    def __init__(self, hz_action, substeps=1, **more_params_that_I_need):
        # Create simulation with higher resolution than actual agent frequency
        self.sim = pb.Simulator(hz_action * substeps)
        self.sim.init('direct')
        self._substeps = substeps

        # Create all your objects

    def reset(self):
        # Do whatever else you need to do
        self.sim.reset()

        # Custom reset behavior such as priming PID-gains
        return self.observation()

    def step(self, action):
        # Post-process your action
        # Send to simulator
        
        # Actual physics. Higher resolution than agent
        for _ in range(self._substeps):
            self.sim.update()

        # Calculate your rewards and done flag
        return self.observation(), reward, done, some_info_dictionary
    
    def observation(self):
        # Do stuff
        return some_observation_that_you_constructed_from_the_sim

    def close(self):
        self.sim.kill()

    @property
    def observation_space(self):
        return some_specification_of_an_observation_space

    @property
    def action_space(self):
        return some_specification_of_an_action_space
```

As you can see in the code example, we suggest that you perform multiple simulation steps per agent step. While this is dependent on your agent's action frequency, we have found that a 30Hz simulation frequency easily leads to oscillations in the simulation. 

## Conclusion

We hope this is enough of an overview to get you started. 
