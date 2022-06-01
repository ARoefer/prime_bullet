import os
import re
import numpy    as np
import pybullet as pb

from dataclasses import dataclass
from hashlib     import md5
from pathlib     import Path
from collections import namedtuple


# Datastructure representing a point
class Point3(tuple):
    def __new__(cls, x, y, z):
        return super(Point3, cls).__new__(cls, (x, y, z))

    def __add__(self, other):
        return Point3(*(np.asarray(self) + other))
    
    def __sub__(self, other):
        if type(other) == Vector3:
            return Point3(*(np.asarray(self) - other))
        return Vector3(*(np.asarray(self) - other))


# Datastructure representing a vector
class Vector3(tuple):
    def __new__(cls, x, y, z):
        return super(Vector3, cls).__new__(cls, (x, y, z))

    def __add__(self, other):
        return Vector3(*(np.asarray(self) + other))
    
    def __sub__(self, other):
        return Vector3(*(np.asarray(self) - other))
    
    def __mul__(self, other):
        return Vector3(*(np.asarray(self) * other))

    def __div__(self, other):
        return Vector3(*(np.asarray(self) / other))

    def __neg__(self):
        return Vector3(*(-np.asarray(self)))

    def dot(self, other):
        return (np.asarray(self) * other).sum()

# Datastructure representing a color
class ColorRGBA(tuple):
    def __new__(cls, r, g, b, a):
        return super(ColorRGBA, cls).__new__(cls, (r, g, b, a))

    def __add__(self, other):
        return ColorRGBA(*(np.asarray(self) + other))
    
    def __sub__(self, other):
        return ColorRGBA(*(np.asarray(self) - other))
    
    def __mul__(self, other):
        return ColorRGBA(*(np.asarray(self) * other))

    def __div__(self, other):
        return ColorRGBA(*(np.asarray(self) / other))

    def __neg__(self):
        return ColorRGBA(*(-np.asarray(self)))

# Datastructure representing a quaternion
class Quaternion(tuple):
    def __new__(cls, x, y, z, w):
        return super(Quaternion, cls).__new__(cls, (x, y, z, w))

    def dot(self, other):
        if type(other) == Quaternion:
            return Quaternion(*pb.multiplyTransforms((0, 0, 0), self,
                                                     (0, 0, 0), other)[1])
        elif type(other) == Vector3:
            return Vector3(*pb.multiplyTransforms((0, 0, 0), self,
                                                  other, (0, 0, 0, 1))[0])
        raise Exception(f'Cannot rotate type {type(other)}')

    def inv(self):
        return Quaternion(*pb.invertTransform((0, 0, 0), self)[1])

    def matrix(self):
        return np.asarray(pb.getMatrixFromQuaternion(self)).reshape((3, 3))

    @staticmethod
    def from_euler(r, p, y):
        return Quaternion(*pb.getQuaternionFromEuler((r, p, y)))

    def euler(self):
        return pb.getEulerFromQuaternion(self)

# Datastructure representing a frame as a Vector3 and a Quaternion
@dataclass
class Transform:
    position   : Point3
    quaternion : Quaternion

    def dot(self, other):
        if type(other) == Transform:
            new_pose = pb.multiplyTransforms(self.position, self.quaternion,
                                             other.position, other.quaternion)
            return Transform(Point3(*new_pose[0]), Quaternion(*new_pose[1]))
        elif type(other) == Vector3:
            return Vector3(*pb.multiplyTransforms((0, 0, 0), self.quaternion,
                                                  other, (0, 0, 0, 1))[0])
        elif type(other) == Point3:
            return Point3(*pb.multiplyTransforms(self.position, self.quaternion,
                                                 other, (0, 0, 0, 1))[0])
        elif type(other) == Quaternion:
            return Quaternion(*pb.multiplyTransforms((0, 0, 0), self.quaternion,
                                                     (0, 0, 0), other)[1])
        raise Exception(f'Cannot transform type {type(other)}')
    
    def inv(self):
        temp = pb.invertTransform(self.position, self.quaternion)
        return Transform(Point3(*temp[0]), Quaternion(*temp[1]))

    def matrix(self):
        out = np.eye(4)
        out[:3, 3]  = self.position
        out[:3, :3] = self.quaternion.matrix()
        return out

    @staticmethod
    def from_xyz_rpy(x, y, z, rr, rp, ry):
        return Transform(Point3(x, y, z), Quaternion.from_euler(rr, rp, ry))

# Axis aligned bounding box structure. Represents AABBs as a tuple of a low and high corner.
@dataclass
class AABB:
    min : Point3
    max : Point3

    def __add__(self, other):
        if type(other) != Vector3:
            raise Exception(f'Cannot add object of type {type(other)} to AABB')
        return AABB(self.min + other, self.max + other)

    def __sub__(self, other):
        if type(other) != Vector3:
            raise Exception(f'Cannot subtract object of type {type(other)} to AABB')
        return AABB(self.min - other, self.max - other)

    @property
    def center(self):
        return self.min + (self.max - self.min) * 0.5


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
