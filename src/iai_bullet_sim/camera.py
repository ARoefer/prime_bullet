import pybullet as pb
import numpy    as np

from iai_bullet_sim.utils import Transform


class Camera(object):
    def __init__(self, simulator, 
                       resolution, 
                       projection_matrix, 
                       near,
                       far,
                       initial_pose : Transform,
                       parent=None, 
                       parent_link=None) -> None:
        self._simulator   = simulator
        self._parent_to_c_pos = None
        self._parent_to_c_rot = None
        self.pose = initial_pose
        self._p_matrix    = projection_matrix.flatten()
        self._resolution  = resolution
        self._parent      = parent
        self._parent_link = parent_link
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
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, pose : Transform):
        self._pose = pose
        self._parent_to_c = self._pose.inv()

    def render(self):
        if self.__last_image_update >= self._simulator.sim_step:
            return

        w_to_c = self._parent_to_c
        if self._parent is not None:
            if self._parent_link is not None:
                link_pose = self._parent.get_link_state(self._parent_link).world_pose
                w_to_c = w_to_c.dot(link_pose.inv())
            else:
                w_to_c = w_to_c.dot(self._parent.pose.inv())

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
                       parent=None, 
                       parent_link=None):
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
                                                parent,
                                                parent_link)
        self._fov = fov

