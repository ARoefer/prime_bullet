import pybullet as pb
import numpy    as np

from iai_bullet_sim.frame    import Frame
from iai_bullet_sim.geometry import Transform


class Camera(Frame):
    def __init__(self, simulator, 
                       resolution, 
                       projection_matrix, 
                       near,
                       far,
                       initial_pose : Transform,
                       parent : Frame = None) -> None:
        super(Camera, self).__init__(parent)
        self._simulator   = simulator
        self.local_pose   = initial_pose
        self._p_matrix    = projection_matrix.flatten()
        self._resolution  = resolution
        self._near        = near
        self._far         = far

        self.initial_pose = initial_pose

        self.__current_rgb = None
        self.__current_d   = None
        self.__current_seg = None
        self.__last_image_update = -1
        self.__view_map = np.asarray(pb.computeViewMatrix([0, 0, 0], 
                                                          [1, 0, 0], 
                                                          [0, 0, 1])).reshape((4, 4)).T

    def reset(self):
        self.__last_image_update = -1
        self.__current_rgb = None
        self.__current_d   = None
        self.__current_seg = None
        self.pose = self.initial_pose

    @property
    def local_pose(self):
        return self._pose

    @local_pose.setter
    def local_pose(self, pose : Transform):
        self._pose = pose

    @property
    def pose(self):
        return super().pose

    @pose.setter
    def pose(self, pose : Transform):
        w_to_p = self.parent.pose.inv() if self.parent is not None else Transform.identity()
        self.local_pose = w_to_p.dot(pose)

    def render(self):
        if self.__last_image_update >= self._simulator.sim_step:
            return

        w_to_c = self.pose.inv()
        vm = self.__view_map.dot(w_to_c.matrix())

        iw, ih, rgba, d, seg = pb.getCameraImage(*self._resolution,
                                                vm.T.flatten(),
                                                self._p_matrix,
                                                renderer=pb.ER_BULLET_HARDWARE_OPENGL,
                                                physicsClientId=self._simulator.client_id)
        rgb   = np.reshape(rgba, (ih, iw, 4))[:, :, :3]
        depth = np.reshape(d, (ih, iw))
        self.__current_rgb = rgb
        self.__current_d   = self._near * self._far / (self._far - (self._far - self._near) * depth)
        self.__current_seg = np.reshape(seg, (ih, iw))

    def rgb(self):
        self.render()
        return self.__current_rgb

    def depth(self):
        self.render()
        return self.__current_d
    
    def rgbd(self):
        self.render()
        return self.__current_rgb, self.__current_d

    def segmentation(self):
        self.render()
        return self.__current_seg


class PerspectiveCamera(Camera):
    def __init__(self, simulator, 
                       resolution, 
                       fov, 
                       near,
                       far,
                       initial_pose : Transform,
                       parent=None):
        super(PerspectiveCamera, self).__init__(simulator, 
                                                resolution,
                                                np.reshape(pb.computeProjectionMatrixFOV(fov, 
                                                                                         resolution[0] / resolution[1], 
                                                                                         near, 
                                                                                         far),
                                                           (4, 4)),
                                                near,
                                                far,
                                                initial_pose,
                                                parent)
        self._fov = fov
    
    def intrinsics(self):
        cx = self._resolution[0] / 2
        cy = self._resolution[1] / 2
        fx = cx / np.tan(np.deg2rad(self._fov) / 2)
        fy = fx
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

