from .basic_simulator import SimulatorPlugin


class ContactEventListener(SimulatorPlugin):
    """docstring for SimulatorPlugin"""
    def __init__(self, bodyA, linkA=None, bodyB=None, linkB=None):
        """
        :type bodyA: iai_bullet_sim.multibody.Multibody, iai_bullet_sim.rigid_body.RigidBody
        :type linkA: str, NoneType
        :type bodyB: iai_bullet_sim.multibody.Multibody, iai_bullet_sim.rigid_body.RigidBody, NoneType
        :type linkB: str, NoneType
        """
        super(ContactEventListener, self).__init__('Contact Event Listener')
        self.bodyA = bodyA
        self.bodyB = bodyB
        self.linkA = linkA
        self.linkB = linkB

    def post_physics_update(self, simulator, deltaT):
        """
        :type simulator: iai_bullet_sim.basic_simulator.BasicSimulator
        :type deltaT: float
        """
        contacts = simulator.get_contacts(self.bodyA, self.bodyB, self.linkA, self.linkB)
        if len(contacts) > 0:
            self.on_contact(contacts)

    def on_contact(self, contacts):
        """
        :type contacts: list
        """
        raise (NotImplementedError)
