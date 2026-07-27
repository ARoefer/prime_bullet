import sys

from pathlib import Path

from .utils         import ColorRGBA, \
                           add_search_path, \
                           res_pkg_path


# Under a plain source import __file__ points at this file, whose parent holds data/. Under the
# catkin devel stub, however, this __init__ is exec'd with __file__ pointing into devel/ (which has
# no data/), so fall back to locating the real source dir -- it is always on sys.path via the stub.
_here = Path(__file__).parent
if (_here / 'data').is_dir():
    IAI_BULLET_ROOT = _here
else:
    IAI_BULLET_ROOT = next((Path(p) / 'prime_bullet' for p in sys.path
                                                     if (Path(p) / 'prime_bullet' / 'data').is_dir()),
                           _here)


from .geometry      import Point3, \
                           Vector3, \
                           Quaternion, \
                           Transform, \
                           AABB, \
                           pb as pybullet

from .frame         import Frame
from .camera        import Camera,            \
                           PerspectiveCamera, \
                           OrthographicCamera
from .laser_scanner import LaserScanner
from .rigid_body    import RigidBody, \
                           BoxBody, \
                           CylinderBody, \
                           SphereBody, \
                           MeshBody, \
                           SDFBody, \
                           SDFWorldBody

from .multibody     import MultiBody, Link


from .simulator     import Simulator,  \
                           ContactPoint,    \
                           SimulatorPlugin, \
                           DebugVisualizer

from .controllers   import JointPositionController, \
                           CartesianController, \
                           CartesianRelativeVirtualPointController,       \
                           CartesianRelativePointController,              \
                           CartesianRelativeController,                   \
                           CartesianRelativePointCOrientationController,  \
                           CartesianRelativeVPointCOrientationController
