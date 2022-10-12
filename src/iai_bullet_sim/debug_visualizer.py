import pybullet as pb
import numpy    as np

from typing      import List
from dataclasses import dataclass

from .geometry   import Point3, Vector3, Quaternion, Transform
from .rigid_body import RigidBody
from .multibody  import MultiBody
from .utils      import ColorRGBA

@dataclass
class DebugVisItem():
    bullet_id : int
    client_id : int

    def __hash__(self):
        return hash(self.bullet_id) ^ (hash(self.client_id) << 1)
    
    def __del__(self):
        pb.removeUserDebugItem(self.bullet_id, self.client_id)

@dataclass
class DebugVisItemBatch():
    bullet_ids : List[int]
    client_id  : int

    def __hash__(self):
        return hash(self.bullet_id) ^ (hash(self.client_id) << 1)
    
    def __del__(self):
        for bid in self.bullet_ids:
            pb.removeUserDebugItem(bid, self.client_id)


class DebugVisualizer(object):
    def __init__(self, client_id, flags= pb.COV_ENABLE_SHADOWS
                                       | pb.COV_ENABLE_RENDERING,
                                  background_color=Vector3(0.7, 0.9, 1.0)):
        self._client_id    = client_id
        self._active_flags = flags
        self._bkg_color    = np.clip(background_color, 0, 1)

        pb.configureDebugVisualizer(~0, 0, rgbBackground=self._bkg_color, physicsClientId=self._client_id)
        pb.configureDebugVisualizer(self._active_flags, 1, physicsClientId=self._client_id)

    def set_flag(self, flag, active=True):
        pb.configureDebugVisualizer(flag, int(active), physicsClientId=self._client_id)

        if active:
            self._active_flags |= flag
        else:
            self._active_flags &= ~flag
        
    def is_active(self, flag):
        return self._active_flags & flag != 0
    
    def set_background_color(self, color):
        self._bkg_color = np.clip(color, 0, 1)
        pb.configureDebugVisualizer(0, 0, rgbBackground=self._bkg_color, physicsClientId=self._client_id)

    def set_camera_position(self, pof : Point3, distance : float, pitch : float, yaw : float):
        pb.resetDebugVisualizerCamera(distance, yaw, pitch, pof, physicsClientId=self._client_id)
    
    def set_camera_pose(self, pose : Transform):
        focal_point = pose.dot(Point3(0, 0, 1))
        
        r, p, y = pose.quaternion.euler()
        pb.resetDebugVisualizerCamera(1, np.rad2deg(y), np.rad2deg(p), focal_point, physicsClientId=self._client_id)

    def get_camera_position(self) -> Transform:
        width, height, view, proj, up, fwd, hor, vert, yaw, pitch, dist, target = pb.getDebugVisualizerCamera(physicsClientId=self._client_id)
        return Point3(*target), dist, pitch, yaw

    def get_camera_pose(self) -> Transform:
        width, height, view, proj, up, fwd, hor, vert, yaw, pitch, dist, target = pb.getDebugVisualizerCamera(physicsClientId=self._client_id)
        rotation = Quaternion.from_euler(np.deg2rad(0), 
                                         np.deg2rad(pitch), 
                                         np.deg2rad(yaw))
        print(rotation.matrix())
        return Transform.from_xyz(*target).dot(Transform(Point3.zero(), rotation).dot(Transform.from_xyz(0, 0, -dist)))

    def draw_line(self, start, end, color=ColorRGBA.blue(), width=0.1, time=0.0, frame=None):
        if type(frame) in {RigidBody, MultiBody}:
            iid = pb.addUserDebugLine(start, end, color, width, time, frame.bId, -1, physicsClientId=self._client_id)
            return DebugVisItem(iid, self._client_id) if time == 0.0 else None
        elif type(frame) == Link:
            iid = pb.addUserDebugLine(start, end, color, width, time, frame.bId, frame.idx, physicsClientId=self._client_id)
            return DebugVisItem(iid, self._client_id) if time == 0.0 else None
        elif type(frame) == Frame:
            pose  = frame.pose    
            start = pose.dot(start)
            end   = pose.dot(end)
        
        iid = pb.addUserDebugLine(start, end, color, width, time, physicsClientId=self._client_id)
        return DebugVisItem(iid, self._client_id) if time == 0.0 else None

    def draw_path(self, points, color=ColorRGBA.blue(), width=0.1, time=0.0, frame=None):
        if frame is not None and type(frame) not in {RigidBody, MultiBody, Link}:
            tf = frame.pose.matrix()
            points = tf.dot(np.hstack((points, np.ones((len( points), 1)))).T)[:,:3]
        
        if type(frame) in {RigidBody, MultiBody, Link}:
            body_id  = frame.bId
            link_idx = -1 if type(frame) != Link else frame.idx
        else:
            body_id  = None
            link_idx = None 

        iids = [pb.addUserDebugLine(p0, p1, color, width, time, 
                                    body_id, link_idx, 
                                    physicsClientId=self._client_id) for p0, p1 in zip(points[:-1], points[1:])]

        return DebugVisItemBatch(iids, self._client_id) if time == 0.0 else None

    def draw_vector(self, vector, color=ColorRGBA.red(), width=0.1, time=0.0, frame=None):
        return self.draw_line(Point3.zero(), vector, color, width, time, frame)
    
    def draw_transform(self, transform=Transform.identity(), width=0.1, size=0.5, time=0.0, frame=None):
        axes = [transform.dot(Point3.unit_x() * size),
                transform.dot(Point3.unit_y() * size),
                transform.dot(Point3.unit_z() * size)]
        
        iids = [self.draw_line(transform.position, 
                               a, c, width, 
                               time, frame) for a, c in zip(axes, [ColorRGBA.red(), 
                                                                   ColorRGBA.green(), 
                                                                   ColorRGBA.blue()])]
        
        return DebugVisItemBatch(iids, self._client_id) if iids[0] is not None else None

    def draw_orientation(self, rotation=Quaternion.identity(), width=0.1, size=0.5, time=0.0, frame=None):
        axes = [rotation.dot(Vector3.unit_x() * size),
                rotation.dot(Vector3.unit_y() * size),
                rotation.dot(Vector3.unit_z() * size)]
        
        iids = [self.draw_vector(a, c, width, 
                                 time, frame) for a, c in zip(axes, [ColorRGBA.red(), 
                                                                     ColorRGBA.green(), 
                                                                     ColorRGBA.blue()])]
        
        return DebugVisItemBatch(iids, self._client_id) if iids[0] is not None else None

    def draw_points(self):
        pass
    
    def draw_text(self):
        pass
