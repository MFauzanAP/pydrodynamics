import math
import numpy as np

from params import ParamsManager
from utils import State, Position, Orientation, LinearVelocity, AngularVelocity, body_to_world, fix_angular_circularity

class Dynamics:
	def __init__(self, params: ParamsManager):
		"""Initialize the dynamics model with parameters and initial state."""
		self.params = params
		self.num_states = 13

		# Construct mass matrix
		# TODO: add added mass coefficients
		m = params.get('mass')
		xg = params.get('xg')
		yg = params.get('yg')
		zg = params.get('zg')
		ixx = params.get('ixx')
		iyy = params.get('iyy')
		izz = params.get('izz')
		X_du = 0.0
		Y_dv = 0.0
		Y_dr = 0.0
		Z_dw = 0.0
		Z_dq = 0.0
		K_dp = 0.0
		M_dw = 0.0
		M_dq = 0.0
		N_dv = 0.0
		N_dr = 0.0
		self.inv_mass_matrix = np.linalg.inv(np.array([
			[m-X_du,	0.0,		0.0, 		0.0, 		m*zg, 		-m*yg],
			[0.0,       m-Y_dv,     0.0,       	-m*zg,     	0.0,       	m*xg-Y_dr],
			[0.0,       0.0,       	m-Z_dw,     m*yg,  		-m*xg-Z_dq, 0.0],
			[0.0,      	-m*zg,      m*yg,    	ixx-K_dp,   0.0,        0.0],
			[m*zg,      0.0,    	-m*xg-M_dw, 0.0,     	iyy-M_dq,   0.0],
			[-m*yg,  	m*xg-N_dv,  0.0,        0.0,       	0.0,     	izz-N_dr]
		]))

	def calculate(self, dt, state, tau):
		"""Calculate the next state of the vehicle based on external forces."""
		if self.params.verbose: print(tau)

		# Unpack state
		x, y, z = state.position.x, state.position.y, state.position.z
		roll, pitch, yaw = state.orientation.roll, state.orientation.pitch, state.orientation.yaw
		u, v, w = state.linear_velocity.u, state.linear_velocity.v, state.linear_velocity.w
		p, q, r = state.angular_velocity.p, state.angular_velocity.q, state.angular_velocity.r

		# Create a new state in np array format
		state = np.array([
			x, y, z,
			roll, pitch, yaw,
			u, v, w,
			p, q, r,
			state.voltage
		])

		# Calculate the next state using Runge-Kutta method
		next_state = self.runge_kutta(dt, state, tau)
		if self.params.verbose: print(next_state)

		# Convert the next state back to State object
		return State(
			position=Position(next_state[0], next_state[1], next_state[2]),
			orientation=Orientation(next_state[3], next_state[4], next_state[5]),
			linear_velocity=LinearVelocity(next_state[6], next_state[7], next_state[8]),
			angular_velocity=AngularVelocity(next_state[9], next_state[10], next_state[11]),
			voltage=next_state[12],
		)

	def runge_kutta(self, dt, state, tau):
		"""
			Runge-Kutta method to solve the dynamics of the vehicle accurately.
			Integrates and approximates the result from solve_dynamics over time.
			
			Input:
				dt: Time step for the simulation.
				state: Current state of the vehicle, in np array format.
				tau: External force and moment due to thrusters in body fixed frame.
			Output:
				next_state: The next state of the vehicle after applying the thruster forces, in np array format. 
		"""
		k1 = self.solve_dynamics(state, tau)
		k2 = self.solve_dynamics(state + k1 * dt / 2, tau)
		k3 = self.solve_dynamics(state + k2 * dt / 2, tau)
		k4 = self.solve_dynamics(state + k3 * dt, tau)

		next_state = state + (k1 + 2*k2 + 2*k3 + k4) * dt / 6
		return fix_angular_circularity(next_state)

	def solve_dynamics(self, state, tau):
		"""
			Solves the kinematics and dynamics of the vehicle, but needs runge-kutta since these are
			non-linear and coupled ODEs. Runge-Kutta will approximate the state over time for a more precise result.

			Input:
				dt: Time step for the simulation.
				state: Current state of the vehicle, in np array format.
				tau: External force and moment due to thrusters in body fixed frame.
			Output:
				delta_state: The change in state of the vehicle after applying the thruster forces, in np array format, denoting acceleration and new velocities.
		"""
		# Solve for accelerations
		accels = np.matmul(self.inv_mass_matrix, tau).reshape((6,))

		# Convert body-fixed velocities to world frame to match world fixed frame for position and orientation when integrating
		vels = body_to_world(state)

		return np.concatenate((vels, accels, [0]), axis=0).reshape((self.num_states,))
