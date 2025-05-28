import numpy as np
import prime_bullet as pb
import random
import signal
import tempfile

from dataclasses import dataclass
from enum    import Enum
from functools import partial
from hashlib import md5
from jinja2  import Template
from pathlib import Path
from scipy.stats.qmc import PoissonDisk
from gmr import MVN

class MaterialDensities(Enum):
    CARDBOARD = 0.69 * 100**3 / 1000 # g/cm^3 to kg/m^3
    WOOD      =  0.7 * 100**3 / 1000 # g/cm^3 to kg/m^3
    ABS       = 1.53 * 100**3 / 1000 # g/cm^3 to kg/m^3
    ALUMINUM  =  2.7 * 100**3 / 1000 # g/cm^3 to kg/m^3
    GRANITE   =  2.7 * 100**3 / 1000 # g/cm^3 to kg/m^3
    STEEL     =  7.8 * 100**3 / 1000 # g/cm^3 to kg/m^3
    BRASS     =  8.6 * 100**3 / 1000 # g/cm^3 to kg/m^3
    COPPER    =  9.0 * 100**3 / 1000 # g/cm^3 to kg/m^3
    LEAD      = 11.3 * 100**3 / 1000 # g/cm^3 to kg/m^3
    GOLD      = 19.3 * 100**3 / 1000 # g/cm^3 to kg/m^3

    @classmethod
    def values(cls):
        return [member.value for member in cls]


class AppState:
    should_stop = False

with open(f'{pb.IAI_BULLET_ROOT}/data/urdf/template_container.urdf', 'r') as f:
    _TEMPLATE_CONTAINER = Template(f.read())

class ContainerBody(pb.RigidBody):
    def __init__(self, simulator : pb.Simulator,
                       size=pb.Vector3(1, 1, 1),
                       initial_pose=pb.Transform.identity(),
                       color=pb.ColorRGBA(1, 0, 0, 1),
                       mass=1,
                       density=None,
                       wall_thickness=0.01,
                       closed=True):
        self.size  = size
        d, w, h = size
        volume = 2 * wall_thickness * (h * w + h * d + (w * d) / (2 - int(closed)))
        self.mass  = mass if density is None else volume * density
        self.density = density if density is not None else mass / volume
        self.color = color
        self.wall_thickness = wall_thickness
        self.contents = []

        file_hash = md5(f'container_{size}_{mass}_{color}_{wall_thickness}_{closed}'.encode('utf-8')).hexdigest()
        fpath = Path(f'{tempfile.gettempdir()}/{file_hash}.urdf')
        if True: # not fpath.exists():
            with open(str(fpath), 'w') as f:
                f.write(_TEMPLATE_CONTAINER.render(mass=mass,
                                                   size=size,
                                                   color=color,
                                                   wall_thickness=wall_thickness,
                                                   closed=closed))
        
        bulletId = pb.pybullet.loadURDF(str(fpath),
                                        initial_pose.position,
                                        initial_pose.quaternion,
                                        0,
                                        0,
                                        pb.pybullet.URDF_MERGE_FIXED_LINKS | pb.pybullet.URDF_ENABLE_SLEEPING,
                                        physicsClientId=simulator.client_id)
        
        super().__init__(simulator, bulletId, 'container', initial_pose)

    @property
    def inner_frame(self) -> pb.Transform:
        return self.pose @ pb.Transform.from_xyz(0, 0, -0.5 * self.size[2] + self.wall_thickness)

    @pb.RigidBody.local_pose.setter
    def local_pose(self, pose):
        before_T_w = self.pose.inv()
        pb.RigidBody.local_pose.__set__(self, pose)

        for c in self.contents:
            c.pose = pose @ before_T_w @ c.pose


    @pb.RigidBody.initial_pose.setter
    def initial_pose(self, pose):
        before_T_w = self.pose.inv()
        pb.RigidBody.initial_pose.__set__(self, pose)

        for c in self.contents:
            c.initial_pose = pose @ before_T_w @ c.pose

    @pb.RigidBody.velocity.setter
    def velocity(self, velocity):
        raise NotImplementedError('This is not correct.')
        pb.RigidBody.velocity.__set__(self, velocity)

        for c in self.contents:
            c.velocity = velocity

    @pb.RigidBody.linear_velocity.setter
    def linear_velocity(self, velocity):
        pb.RigidBody.linear_velocity.__set__(self, velocity)

        for c in self.contents:
            c.linear_velocity = velocity

    @pb.RigidBody.angular_velocity.setter
    def angular_velocity(self, velocity):
        raise NotImplementedError('This is not correct.')
        pb.RigidBody.angular_velocity.__set__(self, velocity)

        for c in self.contents:
            c.angular_velocity = velocity

    def add_contents(self, contents):
        self.contents.extend(contents)

    @property
    def mass_loaded_box(self) -> float:
        return self.mass + sum([c.mass for c in self.contents])


def create_box(sim : pb.Simulator,
               size_sampler : callable,
               wall_thickness_sampler : callable,
               closed_sampler : callable,
               density_sampler : callable,
               n_content_sampler : callable,
               content_position_sampler : callable,
               content_shape_sampler : callable,
               content_size_sampler : callable,
               content_density_sampler : callable,
               ) -> ContainerBody:
    size = size_sampler().squeeze()
    wall_thickness = wall_thickness_sampler()

    box = ContainerBody(sim, size, density=density_sampler(), wall_thickness=wall_thickness, closed=closed_sampler())
    sim.register_object(box)

    n_contents = n_content_sampler()
    if n_contents > 0:
        inner_object_positions = content_position_sampler(20)
        inner_object_positions -= inner_object_positions.mean(axis=0)
        valid_positions_mask   = (np.abs(inner_object_positions) < np.asarray(box.size[:2]) * 0.5 - wall_thickness).all(axis=1)
        inner_object_positions = inner_object_positions[valid_positions_mask]

        n_valid_samples = min(n_contents, valid_positions_mask.sum())

        if n_valid_samples == 0:
            return box

        extents   = content_size_sampler(n_valid_samples, box.size - 2 * wall_thickness)
        densities = content_density_sampler(n_valid_samples)
        shapes    = content_shape_sampler(n_valid_samples)

        inner_objects = []
        if extents[0] is None:
            return box

        for xy_pos, d, shape, size in zip(inner_object_positions, densities, shapes, extents):
        
            if shape == 'box': # Box/Cylinder/Sphere
                inner_obj = sim.create_box(size,
                                            density=d)
                box_P_inner = pb.Point3(xy_pos[0], xy_pos[1], size[2] * 0.5)
            elif shape == 'cylinder':
                inner_obj = sim.create_cylinder(size[0] * 0.5,
                                                size[2],
                                                density=d)
                box_P_inner = pb.Point3(xy_pos[0], xy_pos[1], size[2] * 0.5)
            elif shape == 'sphere':
                inner_obj = sim.create_sphere(size[0] * 0.5, 
                                              density=d)
                box_P_inner = pb.Point3(xy_pos[0], xy_pos[1], size[0] * 0.5)
            else:
                raise ValueError(f'Unknown object type: {shape}')

            inner_obj.pose = box.inner_frame @ pb.Transform.from_xyz(*box_P_inner)
            inner_objects.append(inner_obj)
    
        box.add_contents(inner_objects)

    return box


def limit_sampler(choices : np.ndarray, samples, limits : np.ndarray) -> np.ndarray:
        choice_mask = (choices < limits).all(axis=-1)
        if not choice_mask.any():
            return [None] * samples
        return random.choices(list(choices[choice_mask]), k=samples)

inner_object_sampler = PoissonDisk(2)

def generate_box_scene(sim : pb.Simulator, n_boxes):
    box_sizes = [np.array([0.4, 0.3, 0.25]),
                 np.array([0.4, 0.4, 0.2]),
                 np.array([0.3, 0.3, 0.4]),
                 np.array([0.2, 0.2, 0.1]),
                 np.array([0.2, 0.2, 0.2]),
                 np.array([0.1, 0.1, 0.1])]

    object_sizes = [np.array([0.08, 0.08, 0.3]),
                    np.array([0.12, 0.01, 0.12]),
                    np.array([0.3, 0.05, 0.05]),
                    np.array([0.05, 0.1, 0.13]),
                    np.array([0.2, 0.2, 0.2]),
                    np.array([0.1, 0.1, 0.05])]

    boxes = []

    xy_positions = np.stack(np.meshgrid(np.linspace(-2, 2, 10),
                                        np.linspace(-2, 2, 10)), axis=-1).reshape((-1, 2))
    xy_position_weights = 1 / (np.linalg.norm(xy_positions, axis=-1)**2)

    for x in range(n_boxes):

        box = create_box(sim,
                        lambda: np.array([0.5, 0.5, 0.4]),
                        # partial(random.choice, box_sizes),
                        lambda: 0.01,
                        lambda: True,
                        lambda: MaterialDensities.ABS.value,
                        lambda: np.random.choice(4) + 2,
                        inner_object_sampler.random,
                        partial(np.random.choice, ['box', 'cylinder', 'sphere']),
                        content_size_sampler=partial(limit_sampler, np.asarray(object_sizes)),
                        content_density_sampler=partial(np.random.choice, [MaterialDensities.ALUMINUM.value,
                                                                           MaterialDensities.BRASS.value,
                                                                           MaterialDensities.STEEL.value,
                                                                           MaterialDensities.WOOD.value]))

        if len(boxes) == 0:
            xy_position = xy_positions[np.random.choice(len(xy_positions), p=xy_position_weights / xy_position_weights.sum())]
        else:
            # Boxes' xy locations and their longest xy radius
            box_xys = np.asarray([(box.pose.position.x,
                                   box.pose.position.y,
                                   np.linalg.norm(box.size[:2] * 0.5)) for box in boxes])

            position_mask = (np.linalg.norm(xy_positions[:,None] - box_xys[None,:,:2], axis=-1) > np.linalg.norm(box.size[:2]) + box_xys[:,-1]).all(axis=-1)
            xy_position   = xy_positions[position_mask][np.random.choice(position_mask.sum(), p=(f:=xy_position_weights[position_mask]) / f.sum())]

        box.pose = pb.Transform.from_xyz(*xy_position, box.size[2] * 0.5)
        box.initial_pose = box.pose

        boxes.append(box)
    
    return boxes
    

def stack_boxes(bottom : ContainerBody,
                top : ContainerBody,
                velocity_sampler : callable,
                center_offset_sampler : callable):
    xy_offset = center_offset_sampler()
    velocity  = velocity_sampler()

    w_T_top = bottom.pose @ pb.Transform.from_xyz(*xy_offset, 0.5 * (bottom.size[2] + top.size[2]))
    top.pose = w_T_top
    top.linear_velocity = velocity


def wait_for_rest(sim, objects : list[pb.RigidBody], threshold=1e-3, max_steps=1000):
    for _ in range(5):
        sim.update()
    # Wait for scene to rest
    for s in range(max_steps): 
        if max([np.linalg.norm(o.velocity) for o in objects]) < threshold:
            break
        sim.update()
    else:
        raise RuntimeError(f'Scene did not come to rest after {max_steps} steps')
    print(f'waited {s} steps')


def stack_and_wait(sim : pb.Simulator,
                   box_order : list[ContainerBody],
                   stop_on_fail=True):
    for bottom, top in zip(box_order[:-1], box_order[1:]):
        stack_boxes(bottom, top, lambda: hemispheric_vector_sample(np.pi, np.deg2rad(20)), lambda: np.random.normal(scale=0.05, size=2))

        wait_for_rest(sim, box_order)

def hemispheric_vector_sample(mean_pitch, std_pitch) -> pb.Vector3:
    return pb.Quaternion.from_euler(0, 0, np.random.random() * 2 * np.pi) @ pb.Quaternion.from_euler(0, np.random.normal(mean_pitch, std_pitch), 0) @ pb.Vector3.unit_x()


class MVNScaled(MVN):
    def sample(self, n_samples, space_limit) -> np.ndarray:
        samples = super().sample(n_samples)
        return np.clip(samples, a_min=None, a_max=space_limit)


def main():
    app = AppState()

    def cb_interrupt(*args):
        app.should_stop = True

    signal.signal(signal.SIGINT, cb_interrupt)

    sim = pb.Simulator(real_time=False)
    sim.init('gui')

    ground = sim.create_box((10, 10, 1),
                            pb.Transform.from_xyz(0, 0, -0.5),
                            mass=0, color=(1, 1, 1, 1))
    
    box_base_size = 0.4
    inner_object_sampler = PoissonDisk(2)
    
    size_mvn = MVN(np.array([0.4, 0.4, 0.25]),
                   np.eye(3) * 0.02)

    content_size_mvn = MVNScaled(np.array([0.05, 0.05, 0.2]),
                                 np.diag([0.001, 0.001, 0.01]))

    
    boxes = generate_box_scene(sim, 6)

    _ = input('Hit enter to start')

    while not app.should_stop:
        stack_and_wait(sim, boxes)
        np.random.shuffle(boxes)
        sim.reset()


if __name__ == '__main__':
    main()