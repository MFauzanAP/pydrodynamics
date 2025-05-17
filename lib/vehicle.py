from utils import State, Position, Orientation, LinearVelocity, AngularVelocity
from dynamics import Dynamics
from params import ParamsManager
from thrusters import ThrusterDynamics

class Vehicle:
	def __init__(self, params, initial_state=None):
		self.params = ParamsManager(params)
		self.dynamics = Dynamics(self.params)
		self.thrusters = ThrusterDynamics(self.params)

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
		tau_t = self.thrusters.calculate(pwm_array)
		tau = tau_t

		# Calculate the next state using the dynamics model
		self.state = next_state if next_state else self.dynamics.calculate(dt, self.state, tau)
		print(self.state)

if __name__ == "__main__":
	auv = Vehicle("examples/bpv2.yml")
	auv.step(1, [1100, 1100, 1100, 1100, 1900, 1100, 1100, 1100])
	auv.step(1, [1100, 1100, 1100, 1100, 1900, 1100, 1100, 1100])
	auv.step(1, [1100, 1100, 1100, 1100, 1900, 1100, 1100, 1100])
	auv.step(1, [1100, 1100, 1100, 1100, 1900, 1100, 1100, 1100])
	auv.step(1, [1100, 1100, 1100, 1100, 1900, 1100, 1100, 1100])
