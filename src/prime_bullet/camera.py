import pybullet as pb
import numpy    as np

from .frame    import Frame
from .geometry import Transform


class Camera(Frame):
    def __init__(self, simulator, 
                       resolution, 
                       projection_matrix, 
                       near,
                       far,
                       initial_pose : Transform,
                       parent : Frame = None,
                       hidden_pose : Transform = Transform.identity()) -> None:
        super(Camera, self).__init__(parent)
        self._simulator   = simulator
        self.local_pose   = initial_pose
        self._p_matrix    = projection_matrix.flatten()

        # For generating point clouds
        # self._inv_p_matrix = np.linalg.inv(projection_matrix)
        X, Y = np.meshgrid(np.linspace(-1, 1, resolution[0]), np.linspace(1, -1, resolution[1]))
        self._uv = np.vstack((X.flatten(), Y.flatten()))

        self._resolution  = resolution
        self._near        = near
        self._far         = far
        self._aspect      = resolution[0] / resolution[1]

        # Only used to render pseudo-orthographic cameras.
        # PyBullet is phenomenally stupid sometimes.
        self._hidden_pose = hidden_pose

        self.initial_pose = initial_pose

        self.__current_rgb   = None
        self.__current_d     = None
        self.__current_lin_d = None
        self.__current_seg   = None
        self.__current_pcd   = None
        self.__last_image_update = -1
        self.__view_map = np.asarray(pb.computeViewMatrix([0, 0, 0], 
                                                          [1, 0, 0], 
                                                          [0, 0, 1])).reshape((4, 4)).T
        self.__c_T_pix  = np.linalg.inv(self._p_matrix.reshape(4, 4).T.dot(self.__view_map).dot(self._hidden_pose.matrix()))
        self.__pix_coords = np.stack(np.meshgrid(np.linspace(-1,  1, resolution[0]),
                                                 np.linspace( 1, -1, resolution[1])), -1).reshape(-1, 2).T

    def reset(self):
        self.__last_image_update = -1
        self.__current_rgb   = None
        self.__current_d     = None
        self.__current_lin_d = None
        self.__current_seg   = None
        self.__current_pcd   = None
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

        w_to_c = self.pose.dot(self._hidden_pose).inv()
        vm = self.__view_map.dot(w_to_c.matrix())

        iw, ih, rgba, d, seg = pb.getCameraImage(*self._resolution,
                                                vm.T.flatten(),
                                                self._p_matrix,
                                                renderer=pb.ER_BULLET_HARDWARE_OPENGL,
                                                physicsClientId=self._simulator.client_id)
        rgb   = np.reshape(rgba, (ih, iw, 4))[:, :, :3]
        depth = np.reshape(d, (ih, iw))
        self.__current_rgb = rgb
        self.__current_d   = depth 
        self.__current_lin_d = self._near * self._far / (self._far - (self._far - self._near) * depth)
        self.__current_seg = np.reshape(seg, (ih, iw))
        self.__current_pcd = None

    @property
    def view_matrix(self):
        return self.__view_map.dot(self.pose.dot(self._hidden_pose).inv().matrix())

    def project(self, points : np.ndarray):
        return np.array([[self._resolution[0] / 2,    0, 0, self._resolution[0] / 2],
                         [0, self._resolution[1] / 2, 0, self._resolution[1] / 2],
                         [0,                       0, 0.5 * (self._far - self._near), self._near],
                         [0,                       0, 0, 1]]).dot(self.project_gl(points).T).T

    def project_gl(self, points : np.ndarray):
        pm = np.asarray(self._p_matrix).reshape((4, 4)).T
        projected = pm.dot(self.view_matrix).dot(points.T)
        return (projected / projected[3]).T

    def rgb(self):
        self.render()
        return self.__current_rgb

    def depth(self):
        self.render()
        return self.__current_lin_d
    
    def rgbd(self):
        self.render()
        return self.__current_rgb, self.__current_d

    def segmentation(self):
        self.render()
        return self.__current_seg

    def pointcloud(self):
        """Returns camera-frame pointcloud (n, 4) calculated as here: https://github.com/bulletphysics/bullet3/issues/1924"""
        self.render()
        if self.__current_pcd is None:
            pixels = np.vstack([self.__pix_coords, self.__current_d.flatten(), np.ones(self.__pix_coords.shape[1])])
            # X is forward in our system...
            pixels = pixels.T[pixels[2] < 0.999].T
            pixels[2] = pixels[2] * 2 - 1

            points  = self.__c_T_pix.dot(pixels)
            points /= points[3]
            self.__current_pcd = points.T
        return self.__current_pcd

    def intrinsics(self):
        raise NotImplementedError(f'Intrinsics are not implemented by camera of type "{type(self)}".')


class PerspectiveCamera(Camera):
    def __init__(self, simulator, 
                       resolution, 
                       fov_h, 
                       near,
                       far,
                       initial_pose : Transform,
                       parent=None):
        super(PerspectiveCamera, self).__init__(simulator, 
                                                resolution,
                                                np.reshape(pb.computeProjectionMatrixFOV(fov_h, 
                                                                                         resolution[0] / resolution[1], 
                                                                                         near, 
                                                                                         far),
                                                           (4, 4)),
                                                near,
                                                far,
                                                initial_pose,
                                                parent)
        self._fov = fov_h
    
    def intrinsics(self):
        cx, cy = np.asarray(self._resolution) / 2
        fx = cx / np.tan(np.deg2rad(self._fov) / 2)
        fy = cy / np.tan(np.deg2rad(self._fov * (self._resolution[1] / self._resolution[0])) / 2)
        return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])


class OrthographicCamera(Camera):
    """FAKE Orthographic Camera. PyBullet does not support orthographic cameras.
       (https://github.com/bulletphysics/bullet3/issues/2628)
    
       This camera implementation fakes an orthograpic camera by moving the camera
       very far away and increasing the focal length by a factor of 20 compared to
       distance between near and far plane. Use this carefully. It is not numerically
       stable for areas of large depth that you want to capture.
    """
    def __init__(self, simulator, 
                       resolution, 
                       fov_h, 
                       near,
                       far,
                       initial_pose : Transform,
                       parent=None):
        aspect = resolution[0] / resolution[1]
        fov_v  = fov_h / aspect

        new_far = 20 * (far - near)

        # correct_projection = np.asarray([[2 / fov_h,                  0,                 0,            0],
        #                                  [        0, 2 * aspect / fov_h,                 0,            0],
        #                                  [        0,                  0, -2 / (far - near), -(far + near) / (far - near)],
        #                                  [        0,                  0,                 0,            1]]),

        pseudo_ortho = np.asarray(pb.computeProjectionMatrix(-fov_h * 0.5, fov_h * 0.5, -fov_v * 0.5, fov_v * 0.5, new_far - near, new_far)).reshape((4, 4))

        super(OrthographicCamera, self).__init__(simulator, 
                                                 resolution,
                                                 pseudo_ortho,
                                                 near,
                                                 far,
                                                 initial_pose,
                                                 parent,
                                                 hidden_pose=Transform.from_xyz(far - new_far, 0, 0))
        self._fov = fov_h
