import numpy as np
import pybullet as pb

from dataclasses import dataclass

# Datastructure representing a vector
class Vector3(tuple):
    def __new__(cls, x, y, z):
        return super(Vector3, cls).__new__(cls, (x, y, z))

    def __add__(self, other):
        return type(self)(*(np.asarray(self) + other))
    
    def __sub__(self, other):
        return type(self)(*(np.asarray(self) - other))
    
    def __mul__(self, other):
        return type(self)(*(np.asarray(self) * other))

    def __truediv__(self, other):
        return type(self)(*(np.asarray(self) / other))

    def __neg__(self):
        return type(self)(*(-np.asarray(self)))

    def __le__(self, other):
        return np.asarray(self) <= other
    
    def __lt__(self, other):
        return np.asarray(self) <  other
    
    def __ge__(self, other):
        return np.asarray(self) >= other
    
    def __gt__(self, other):
        return np.asarray(self) >  other

    def __eq__(self, other):
        return np.asarray(self) == other

    def __and__(self, other):
        return np.asarray(other) & other
    
    def __or__(self, other):
        return np.asarray(other) | other
    
    def __xor__(self, other):
        return np.asarray(other) ^ other

    def __str__(self):
        return self.__repr__()
    
    def __repr__(self):
        return f'Vector3{super().__repr__()}'

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]
    
    @property
    def z(self):
        return self[2]

    def normalized(self):
        return type(self)(*(self / self.norm()))

    def norm(self):
        return np.sqrt((np.asarray(self) ** 2).sum())

    def dot(self, other):
        return (np.asarray(self) * other).sum()

    def cross(self, other):
        return Vector3(*np.cross(self, other))

    def numpy(self):
        return np.asarray(self)

    @classmethod
    def zero(cls):
        return cls(0, 0, 0)
    
    @classmethod
    def unit_x(cls):
        return cls(1, 0, 0)
    
    @classmethod
    def unit_y(cls):
        return cls(0, 1, 0)

    @classmethod
    def unit_z(cls):
        return cls(0, 0, 1)

# Datastructure representing a point
class Point3(Vector3):
    def __new__(cls, x, y, z):
        return super(Point3, cls).__new__(cls, x, y, z)
    
    def __sub__(self, other):
        if type(other) == Vector3:
            return Point3(*(np.asarray(self) - other))
        return Vector3(*(np.asarray(self) - other))

    def __str__(self):
        return self.__repr__()
    
    def __repr__(self):
        return f'Point3{tuple.__repr__(self)}'

# Datastructure representing a quaternion
class Quaternion(tuple):
    def __new__(cls, x, y, z, w):
        return super(Quaternion, cls).__new__(cls, (x, y, z, w))

    def __str__(self):
        return self.__repr__()
    
    def __repr__(self):
        return f'Quaternion{super().__repr__()}'

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]
    
    @property
    def z(self):
        return self[2]

    @property
    def w(self):
        return self[2]

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
    
    def axis_angle(self, epsilon=1e-4):
        angle = self.angle()
        if angle <= epsilon:
            return Vector3(1, 0, 0), 0.0

        axis  = Vector3(*self[:3])
        axis /= axis.norm()
        return axis, angle

    def angle(self, other=None):
        if other is None:
            return 2 * np.arccos(self[3])

        qd = self.inv().dot(other)
        return 2 * np.arccos(qd[3])

    def lerp(self, other, fac, epsilon=1e-4):
        fac = np.clamp(fac, 0.0, 1.0)
        qd  = self.inv().dot(other)
        ang = 2 * np.arccos(qd[3])
        
        if ang <= epsilon:
            return self

        axis  = Vector3(*qd[:3])
        
        qi = Quaternion.from_axis_angle(axis, ang * fac)
        return self.dot(qi)

    def numpy(self):
        return np.asarray(self)

    @staticmethod
    def from_euler(r, p, y):
        return Quaternion(*pb.getQuaternionFromEuler((r, p, y)))

    @staticmethod
    def from_axis_angle(axis : Vector3, angle : float):
        if axis.norm() <= 1e-4:
            return Quaternion.identity()

        axis /= axis.norm()
        axis *= np.sin(angle * 0.5)
        return Quaternion(*axis, np.cos(angle * 0.5))

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

    def __str__(self):
        return self.__repr__()
    
    def __repr__(self):
        return f'Transform({self.position}, {self.quaternion})'

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

    def relative(self, other):
        return self.inv().dot(other)

    def lerp(self, other, fac, epsilon=1e-4):
        td = self.relative(other)

        axis, angle = td.quaternion.axis_angle(epsilon)
        if td.position.norm() <= epsilon or angle <= epsilon:
            return self

        # Interpolate
        fac = np.clip(fac, 0.0, 1.0)
        ti  = Transform(td.position * fac, Quaternion.from_axis_angle(axis, angle * fac))
        return self.dot(ti)

    def array(self):
        return np.hstack((self.position, self.quaternion))

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

    def inside(self, point : Point3):
        return ((self.min <= point) & (self.max >= point)).min()

    def numpy(self):
        return np.vstack((self.min, self.max))