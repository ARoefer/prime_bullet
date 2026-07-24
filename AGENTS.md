# prime_bullet — reference for agents

Object-oriented wrapper around PyBullet for small manipulation/physics experiments.
**Read this file instead of the source.** It is written to be sufficient for writing working
simulations without opening `src/prime_bullet/*.py`. The last section lists what is broken —
check it before debugging anything surprising.

`README.md` is a human tutorial and contains several errors; where the two disagree, this file wins.

## Environment

- Run everything with the workspace interpreter: `pyenv/bin/python` (at the `lh_ws` root).
- **A stale non-editable copy may be installed at `~/.local/lib/python3.10/site-packages/prime_bullet`.**
  It shadows the source tree and dies on a missing `data/urdf/template_box.urdf`. If you hit that,
  either `pip uninstall prime_bullet` or run with `PYTHONPATH=src/prime_bullet/src`.
- Single import surface: `import prime_bullet as pb`.

## Simulator lifecycle

```python
import prime_bullet as pb

sim = pb.Simulator(step_frequency=60)   # gravity=[0,0,-9.81], real_time=False, use_egl=False
sim.init('direct')                      # or 'gui'

# ... create objects ...

while running:
    sim.update()                        # one *user* tick
    if episode_over:
        sim.reset()                     # restores every body to its initial pose/joint state

sim.stop()                              # only needed with plugins; calls disable() on each
sim.kill()                              # closes the Bullet connection
```

- `step_frequency` **must divide 240** and be `<= 240` (10, 20, 24, 30, 48, 60, 80, 120, 240).
  Constructor raises otherwise. One `update()` runs `240 // step_frequency` physics substeps.
- `sim.sim_step` counts *user ticks*, not substeps, and is the cache key for all lazily-fetched
  state (body poses, link states, joint states). Anything read twice within one tick is free.
- Several `Simulator` instances can coexist; each `init()` claims its own client id.
- `sim.visualizer` is a `DebugVisualizer` in `'gui'` mode, otherwise `None`.
- Sensors (cameras, laser scanners) are **not** registered with the simulator — `sim.reset()`
  does not touch them. Call their own `reset()` if needed.

## Geometry

`Vector3`, `Point3` and `Quaternion` subclass `tuple`, so they pass straight into both pybullet
and numpy with no conversion, while still obeying SE3/SO3 rules. `Transform` is a dataclass.

```python
from prime_bullet import Vector3, Point3, Quaternion, Transform, AABB

p = Point3(1, 2, 3);  v = Vector3(0, 0, 1)
p + v            # Point3      (Point ± Vector -> Point)
p - Point3(...)  # Vector3     (Point - Point  -> Vector)
v.norm(); v.normalized(); v.dot(o); v.cross(o); v.numpy(); v.x; v[0]
Vector3.zero(); Vector3.unit_x(); Vector3.unit_y(); Vector3.unit_z()

q = Quaternion.from_rpy(r, p_, y)            # or .from_euler(r, p, y)
q = Quaternion.from_axis_angle(axis, angle)  # also .from_matrix(m), .identity()
q.dot(other_q); q.dot(vec)   # compose / rotate      q @ other  == q.dot(other)
q.inv(); q.matrix(); q.euler(); q.angle(); q.angle(other); q.axis_angle(); q.lerp(o, f)

t = Transform(position, quaternion)          # NOTE the order: position FIRST
t = Transform.from_xyz(x, y, z)              # .from_rpy, .from_xyz_rpy, .from_xyz_quat,
                                             # .from_matrix, .identity
t.dot(other)     # accepts Transform | Point3 | Vector3 | Quaternion;  t @ other works too
t.inv(); t.matrix(); t.relative(o); t.lerp(o, f); t.array()   # array() -> (7,) xyz+xyzw

aabb.min; aabb.max; aabb.center; aabb.inside(point); aabb.numpy()
```

Quaternions normalize on construction. `Transform` composition is delegated to Bullet.

## Frames

Everything spatial derives from `Frame`, which supplies `local_pose` (relative to `parent`)
and `pose` (world). Bullet has no frame concept, so for bodies and links the two are identical —
the abstraction exists so virtual sensors can be rigidly attached to simulated geometry via
`parent=`. Setting `.pose` on a parented sensor back-computes its `local_pose`.

## Creating objects

```python
box  = sim.create_box([0.2]*3, pb.Transform.from_xyz(0, 1, 0.2), mass=1, color=[1,0,0,1])
sph  = sim.create_sphere(0.3, pose, mass=1)
cyl  = sim.create_cylinder(radius=0.2, height=1.0, pose=pose)
mesh = sim.create_mesh('package://prime_bullet/meshes/suzanne.dae', scale=1, pose=pose,
                       collision_mesh_path=None)   # density= is NOT supported here
tri  = sim.create_mesh_from_trimesh(trimesh_obj, pose=pose)   # needs the [mesh-conversion] extra

robot = sim.load_urdf('package://prime_bullet/urdf/windmill.urdf', pose,
                      useFixedBase=1, use_self_collision=None)  # 'all'|'all_no_parent'|'no_parents'
objs  = sim.load_sdf('model://something')   # returns a LIST
```

- Primitives accept **either** `mass=` **or** `density=` (density wins; mass is derived from volume).
- Primitives are built by rendering Jinja2 URDF templates into `/tmp`, cached by parameter hash.
- `mass=0` makes a body static.
- Lookup/removal go through the simulator by **name**: `sim.bodies` (dict), `sim.get_body(name)`,
  `sim.get_body_id(bullet_id)`, `sim.delete_body(name)`.

## RigidBody

```python
b.pose                    # get/set (setter teleports)          b.local_pose likewise
b.initial_pose            # get/set; what reset() restores      b.reset()
b.velocity                # -> (linear, angular)                b.linear_velocity / b.angular_velocity
b.aabb                    # AABB in world frame
b.bId; b.type
b.dynamics_info.mass      # also lateral_friction, rolling/spinning_friction, restitution,
                          # contact_damping, contact_stiffness, inertia, collision_margin — all settable
b.apply_force(force, point); b.apply_local_force(...)
b.apply_torque(torque);      b.apply_local_torque(...)
b.get_contacts(other=None); b.get_closest_points(other=None, dist=0.2)
b.sleep(); b.wake_up()
```

## MultiBody, Link, Joint

`MultiBody` extends `RigidBody`, so everything above still applies.

```python
r.links['head']; r.i_links[1]; r.get_link('head'); r.get_link_by_index(-1)  # -1 == base
r.joints['head_pan']; r.i_joints[0]
r.base_link; r.joint_names; r.dynamic_joint_names   # dynamic_joint_names fixes array ordering

r.q; r.q_dot; r.q_f          # np.arrays over dynamic joints, ordered by dynamic_joint_names
r.joint_state                # dict name -> JointState(position, velocity, reactionForce, effort)
r.q_min; r.q_max; r.q_dot_max; r.q_f_max

r.set_joint_positions(q_or_dict, override_initial=False)   # teleports, no dynamics
r.apply_joint_pos_cmds(cmd, max_force=None)                # cmd: np.array or dict
r.apply_joint_vel_cmds(cmd, max_force=None)
r.apply_joint_torque_cmds(cmd)                             # toggles torque mode automatically

ft = r.get_ft_sensor('some_joint')    # works on fixed joints too
ft.get()                              # ReactionForces(linear, angular) in joint frame
ft.get_world_space()
```

`Link`: `.name .idx .bId .pose .aabb .dynamics_info .state` where `state` is
`LinkState(com_pose, local_inertial_pose, world_pose, linear_velocity, angular_velocity)`.
Also `link.jacobian(q, q_dot, q_ddot, point)` and `link.ik(world_pose_or_point, max_iterations=50)`,
which returns a q vector for **all** dynamic joints of the body.

`Joint`: `.name .index .type .is_dynamic .limits .q_min .q_max .f_max .qd_max .axis .link .parent`.

Contacts: `sim.get_contacts()`, `sim.get_closest_points(dist)`, `sim.get_overlapping(aabb)`,
`sim.update_collisions()` (collision detection without stepping). Contacts come back as
`ContactPoint(bodyA, bodyB, posOnA, posOnB, normalOnB, dist, normalForce)` where the bodies are
resolved back to `RigidBody`/`Link` objects.

## Sensors

```python
cam = pb.PerspectiveCamera(sim, (200, 200), 70, 0.1, 10.0, pose, parent=link)  # fov in DEGREES
ort = pb.OrthographicCamera(sim, (200, 200), 2.0, 0.1, 10.0, pose)             # 2.0 == view WIDTH in metres
cam.rgb(); cam.depth(); cam.rgbd(); cam.gl_depth(); cam.segmentation()
cam.pointcloud()          # (n, 4) in CAMERA frame, far-plane pixels dropped
cam.intrinsics(); cam.projection_matrix; cam.view_matrix; cam.project(pts); cam.project_gl(pts)

scan = pb.LaserScanner(sim, min_rad, max_rad, steps, range_min, range_max, pose, parent=None)
scan.render()             # (steps,) distances; range_max means "no hit"
```

- **A camera looks along its own local −Z, with image-up along local +X.** (Verified empirically;
  the "X is forward" comment inside `pointcloud()` is stale and wrong.)
- `OrthographicCamera` fakes parallel projection by pulling itself far back along −Z behind a narrow
  frustum, so it is not numerically stable over very large depth ranges — keep `near`/`far` tight.
  Its third argument is the view **width in world units**, not an angle. `depth()` and `pointcloud()`
  are corrected back to the nominal (un-pulled-back) pose, so they read like any other camera.
- Images are re-rendered on **every** `rgb()`/`depth()`/`segmentation()` call, so moving a camera and
  re-reading within one tick works, but repeated reads are not free — cache them yourself.
- Laser rays lie in the scanner's local XY plane, sweeping about +Z.

## Controllers

All expose `act(...)`, `reset()` (adopt current state as goal), `.goal` and `.delta`. All the
Cartesian ones are IK-based and drive position commands.

`JointPositionController(robot)` · `CartesianController(robot, link)` ·
`CartesianRelativeController` · `CartesianRelativePointController` ·
`CartesianRelativePointCOrientationController` · `CartesianRelativeVirtualPointController` ·
`CartesianRelativeVPointCOrientationController`

## Debug drawing (GUI only)

`v = sim.visualizer` → `draw_line`, `draw_path`, `draw_vector`, `draw_transform`,
`draw_orientation`, `set_camera_pose`, `get_camera_pose`, `set_flag`, `set_background_color`.
With `time=0.0` these return a handle that **erases the drawing when garbage collected** — keep a
reference. `draw_points` and `draw_text` are empty stubs.

## Asset paths

`package://<pkg>/<rel>` is resolved against a set of search paths: prime_bullet's own package root
(always registered, so `package://prime_bullet/...` works with zero setup) plus every entry of
`ROS_PACKAGE_PATH`, plus anything you add via `pb.add_search_path(path)`. Candidates are tried in
sorted order and **the first one that exists on disk wins**; if none exist you get an error listing
everything that was checked.

Because Bullet supports only one search path, loaded URDF/SDF files are rewritten into `/tmp` with
absolute paths, missing `<inertial>` blocks injected, and non-`.obj/.stl/.dae` meshes converted via
trimesh.

Shipped assets: `package://prime_bullet/urdf/{windmill,scale}.urdf` and
`package://prime_bullet/meshes/*.dae` (incl. `suzanne.dae`).

## Known broken / removed — check here first

| Thing | Status |
|---|---|
| `save_world` / `load_world` / `save_simulator` / `load_simulator` | raise `NotImplementedError`; unmaintained |
| Constraints (`createConstraint` wrappers, `Constraint` class) | **removed**; use raw pybullet if you need welds |
| `plugins.py` / `ContactEventListener` | **removed**. `SimulatorPlugin` still exists in `simulator.py` and works via `sim.register_plugin()` |
| `scripts/tutorial.py` | stale API (`load_mesh`, `pos=`, `joint_state()` as a method); will not run as written |
| URDF links with no `<inertial>` | get `mass=0` injected, i.e. load **static**. If a robot refuses to move, check `link.dynamics_info.mass` first. |

`README.md` specifically gets these wrong:
`Transform(quaternion, position)` argument order (position comes first), `JointPositionControl`
(it is `JointPositionController`), and `joint_state()` / `pose()` as methods (both are properties).
