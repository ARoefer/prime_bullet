import numpy as np
import pybullet as pb

from dataclasses import dataclass

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

    @staticmethod
    def zero():
        return Point3(0, 0, 0)

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

    @staticmethod
    def zero():
        return Vector3(0, 0, 0)

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

    def euler(self):
        return pb.getEulerFromQuaternion(self)
    
    @staticmethod
    def from_euler(r, p, y):
        return Quaternion(*pb.getQuaternionFromEuler((r, p, y)))

    @staticmethod
    def from_matrix(mat):
        """Extracts a quaternion from a >= 3x3 matrix."""
        w  = np.sqrt(1 + mat[0,0] + mat[1,1] + mat[2,2]) * 0.5
        w4 = 4 * w
        x  = (mat[2,1] - mat[1,2]) / w4
        y  = (mat[0,2] - mat[2,0]) / w4
        z  = (mat[1,0] - mat[0,1]) / w4
        return Quaternion(x,y,z,w)

    @staticmethod
    def identity():
        return Quaternion(0, 0, 0, 1)

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

    @staticmethod
    def from_xyz(x, y, z):
        return Transform(Point3(x, y, z), Quaternion.identity())

    @staticmethod
    def identity():
        return Transform(Point3(0, 0, 0), Quaternion.identity())

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
