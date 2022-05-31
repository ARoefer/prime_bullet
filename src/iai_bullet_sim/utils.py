import os
import re

from hashlib     import md5
from pathlib     import Path
from collections import namedtuple

# Datastructure representing a vector
Vector3 = namedtuple('Vector3', ['x', 'y', 'z'])
# Datastructure representing a point
Point3 = namedtuple('Point3', ['x', 'y', 'z'])
# Datastructure representing a quaternion
Quaternion = namedtuple('Quaternion', ['x', 'y', 'z', 'w'])
# Datastructure representing a frame as a Vector3 and a Quaternion
Pose  = namedtuple('Pose', ['position', 'quaternion'])
# Axis aligned bounding box structure. Represents AABBs as a tuple of a low and high corner.
AABB = namedtuple('AABB', ['min', 'max'])

def rot3_to_quat(rot3):
    """Extracts a quaternion from a >= 3x3 matrix."""
    w  = sp.sqrt(1 + rot3[0,0] + rot3[1,1] + rot3[2,2]) * 0.5
    w4 = 4 * w
    x  = (rot3[2,1] - rot3[1,2]) / w4
    y  = (rot3[0,2] - rot3[2,0]) / w4
    z  = (rot3[1,0] - rot3[0,1]) / w4
    return Quaternion(x,y,z,w)


def res_pkg_path(rpath):
    """Resolves a ROS package relative path to a global path.

    :param rpath: Potential ROS URI to resolve.
    :type rpath: str
    :return: Local file system path
    :rtype: str
    """
    if rpath[:10] == 'package://':
        paths = os.environ['ROS_PACKAGE_PATH'].split(':')

        rpath = rpath[10:]
        pkg = rpath[:rpath.find('/')]

        for rpp in paths:
            if rpp[rpp.rfind('/') + 1:] == pkg:
                return '{}/{}'.format(rpp[:rpp.rfind('/')], rpath)
            if os.path.isdir('{}/{}'.format(rpp, pkg)):
                return '{}/{}'.format(rpp, rpath)
        raise Exception('Package "{}" can not be found in ROS_PACKAGE_PATH!'.format(pkg))
    return rpath


def import_class(class_path):
    """Imports a class using a type string.

    :param class_path: Type string of the class.
    :type  class_path: str
    :rtype: type
    """
    components = class_path.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

RESOLVED_FILES = {}

def abs_urdf_paths(file_path, temp_dir):
    abs_path = Path(res_pkg_path(file_path)).absolute()
    
    if hash(abs_path) in RESOLVED_FILES:
        return RESOLVED_FILES[hash(abs_path)]

    hex_name = md5(str(abs_path).encode('utf-8')).hexdigest()
    temp_file_name = f'{temp_dir}/{hex_name}.urdf'

    with open(file_path, 'r') as of:
        with open(temp_file_name, 'w') as tf:
            for l in of:
                idx = l.find('package://', 0)
                while idx != -1:
                    e_idx = l.find('"', idx)
                    pkg_path = l[idx:e_idx]
                    r_path = res_pkg_path(pkg_path)
                    l = l.replace(l[idx:e_idx], r_path)
                    idx = l.find('package://', idx + len(r_path))
                tf.write(l)
    
    RESOLVED_FILES[abs_path] = temp_file_name

    return temp_file_name
