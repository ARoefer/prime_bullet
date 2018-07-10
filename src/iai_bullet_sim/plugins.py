from iai_bullet_sim.basic_simulator import SimulatorPlugin


class ContactEventListener(SimulatorPlugin):
	"""docstring for SimulatorPlugin"""
	def __init__(self, bodyA, linkA=None, bodyB=None, linkB=None):
		super(ContactEventListener, self).__init__('Contact Event Listener')
		self.bodyA = bodyA
		self.bodyB = bodyB
		self.linkA = linkA
		self.linkB = linkB

	def post_physics_update(self, simulator, deltaT):
		contacts = simulator.get_contacts(self.bodyA, self.bodyB, self.linkA, self.linkB)
		if len(contacts) > 0:
			self.callback(contacts)

	def on_contact(self, contacts):
		raise (NotImplementedError)
		