import numpy as np

from lib.params import ParamsManager
from lib.utils import body_to_world, fix_angular_circularity, state_array_to_object, state_object_to_array

class Dynamics:
    """
        Given the external forces and moments acting on the vehicle, this class calculates the next state of the vehicle.
        Uses Runge-Kutta method to integrate the equations of motion, obtaining position and velocity.
    """
    def __init__(self, params: ParamsManager):
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
        am_xu = params.get('am_xu')
        am_yv = params.get('am_yv')
        am_yr = params.get('am_yr')
        am_zw = params.get('am_zw')
        am_zq = params.get('am_zq')
        am_kp = params.get('am_kp')
        am_mw = params.get('am_mw')
        am_mq = params.get('am_mq')
        am_nv = params.get('am_nv')
        am_nr = params.get('am_nr')
        self.inv_mass_matrix = np.linalg.inv(np.array([
            [m-am_xu,	0.0,		0.0, 			0.0, 		m*zg, 		    -m*yg],
            [0.0,       m-am_yv,    0.0,       		-m*zg,     	0.0,       	    m*xg-am_yr],
            [0.0,       0.0,       	m-am_zw,    	m*yg,  		-m*xg-am_zq,    0.0],
            [0.0,      	-m*zg,      m*yg,    		ixx-am_kp,  0.0,            0.0],
            [m*zg,      0.0,    	-m*xg-am_mw, 	0.0,     	iyy-am_mq,      0.0],
            [-m*yg,  	m*xg-am_nv, 0.0,        	0.0,       	0.0,     	    izz-am_nr]
        ]))

    def calculate(self, dt, state, tau):
        """Calculate the next state of the vehicle based on external forces."""
        if self.params.verbose: print(tau)

        # Create a new state in np array format
        state = state_object_to_array(state)

        # Calculate the next state using Runge-Kutta method
        next_state = self.runge_kutta(dt, state, tau)
        if self.params.verbose: print(next_state)

        # Convert the next state back to State object
        return state_array_to_object(next_state)

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
        # Unpack state
        u, v, w, p, q, r = state[6:12]

        # Solve for accelerations
        accels = np.matmul(self.inv_mass_matrix, tau).reshape((6,))

        # Convert body-fixed velocities to world frame to match world fixed frame for position and orientation when integrating
        r_trans, r_rot = body_to_world(state)
        vels = np.concatenate((
            np.matmul(r_trans, np.array([u, v, w])),
            np.matmul(r_rot, np.array([p, q, r]))
        ), axis=0).reshape((6,))

        return np.concatenate((vels, accels, [0]), axis=0).reshape((self.num_states,))
