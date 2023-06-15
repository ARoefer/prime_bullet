import pybullet as pb

class Constraint(object):
    def __init__(self, simulator, bulletId, type, parent, child, 
                       parent_pos=[0,0,0], parent_rot=[0,0,0,1], 
                       child_pos=[0,0,0], child_rot=[0,0,0,1],
                       axis_in_child=[1,0,0], parent_link=None, child_link=None):
        self.simulator   = simulator
        self.__client_id = simulator.client_id()
        self.__bulletId  = bulletId
        self.type        = type

        self.parent      = parent
        self.child       = child
        self.parent_link = parent_link
        self.child_link  = child_link
        self.max_force   = 1000

        self.parent.register_deletion_cb(self.on_body_deleted)
        if self.child is not None:
            self.child.register_deletion_cb(self.on_body_deleted)

        self.parent_pos = parent_pos
        self.parent_rot = parent_rot
        self.child_pos  = child_pos
        self.child_rot  = child_rot

        self.axis = axis_in_child

    @property
    def bId(self):
        return self.bulletId

    def set_joint_transform(self, child_pos=None, child_rot=None):
        if child_pos is not None or child_rot is not None:
            self.child_pos = child_pos if child_pos is not None else self.child_pos
            self.child_rot = child_rot if child_rot is not None else self.child_rot
            pb.changeConstraint(self.__bulletId, self.child_pos, self.child_rot, self.max_force, physicsClientId=self.__client_id)

    def on_body_deleted(self, simulator, Id, obj):
        if self.parent.bId == obj.bId:
            if self.child is not None:
                self.child.deregister_deletion_cb(self.on_body_deleted)    
        else:
            self.parent.deregister_deletion_cb(self.on_body_deleted)
        
        simulator.delete_constraint(self.bId)