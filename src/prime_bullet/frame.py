from .geometry import Transform


class Frame(object):
    def __init__(self, parent):
        self._parent = parent

    @property
    def parent(self):
        return self._parent

    @property
    def local_pose(self) -> Transform:
        raise NotImplementedError

    @property
    def pose(self) -> Transform:
        return self._parent.pose.dot(self.local_pose) if self._parent is not None else self.local_pose
