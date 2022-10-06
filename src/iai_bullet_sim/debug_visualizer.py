import pybullet as pb
import numpy    as np

from .geometry import Point3, Vector3, Quaternion, Transform

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

    def set_camera_location(self, pof : Point3, distance : float, pitch : float, yaw : float):
        pb.resetDebugVisualizerCamera(distance, yaw, pitch, pof, physicsClientId=self._client_id)
    
    def set_camera_pose(self, pose : Transform):
        focal_point = pose.dot(Point3(0, 0, 1))
        
        r, p, y = pose.quaternion.euler()
        pb.resetDebugVisualizerCamera(1, y, p, focal_point, physicsClientId=self._client_id)

