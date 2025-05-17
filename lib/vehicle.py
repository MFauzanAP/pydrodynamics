from lib.utils import State, Position, Orientation, LinearVelocity, AngularVelocity
from lib.dynamics import Dynamics
from lib.params import ParamsManager
from lib.thrusters import Thrusters
from lib.environment import Environment

class Vehicle:
	"""
		Main class tying all the different modules together.
		It initializes the vehicle from the given yaml file and calculates the next state of the vehicle given PWM control signals.
	"""
	def __init__(self, params, initial_state: State = None):
		self.params = ParamsManager(params)
		self.dynamics = Dynamics(self.params)
		self.thrusters = Thrusters(self.params)
		self.env = Environment(self.params)

		self.state = initial_state if initial_state else State(
			position=Position(0, 0, 0),
			orientation=Orientation(0, 0, 0),
			linear_velocity=LinearVelocity(0, 0, 0),
			angular_velocity=AngularVelocity(0, 0, 0),
			voltage=self.params.get('voltage'),
		)

	def step(self, dt, pwm_array, next_state=None):
		"""
			Calculate the next state of the vehicle.

			Input:
				dt: Time step for the simulation.
				pwm_array: Array of PWM values for the thrusters.
				next_state: If given, this will override the current state, used for collision detection.
			Output:
				next_state: The next state of the vehicle after applying the thruster forces.
		"""
		# Calculate total external forces and moments
		tau_thrusters = self.thrusters.calculate(self.state, pwm_array)
		tau_env = self.env.calculate(self.state)
		tau = tau_thrusters + tau_env

		# Calculate the next state using the dynamics model
		self.state = next_state if next_state else self.dynamics.calculate(dt, self.state, tau)
		print(self.state)
		return self.state
