import numpy as np

from lib.params import ParamsManager
from lib.utils import unpack_state_object
from scipy.interpolate import RegularGridInterpolator

class Thrusters:
    """
        Find thruster forces and moments based on PWM values.
        Uses thruster data from the given CSV file and interpolates with the current voltage.
        Also takes into account the thruster positions and directions used in the vehicle model.
    """
    def __init__(self, params: ParamsManager):
        self.params = params

        self.num_thrusters = len(params.thrusters.list)
        self.tam = np.zeros((6, self.num_thrusters))
        self.thruster_positions = np.zeros((self.num_thrusters, 3))
        self.thruster_directions = np.zeros((self.num_thrusters, 3))
        for i in range(self.num_thrusters):
            # Construct thruster allocation matrix
            self.tam[0:3, i] = np.array([params.get(f't{i + 1}_dir_x'),
                                        params.get(f't{i + 1}_dir_y'),
                                        params.get(f't{i + 1}_dir_z')])
            self.tam[3:6, i] = np.cross(np.array([params.get(f't{i + 1}_x') + params.get('xg'),
                                                params.get(f't{i + 1}_y') + params.get('yg'),
                                                params.get(f't{i + 1}_z') + params.get('zg')]),
                                        self.tam[0:3, i])
            
            # Construct thruster position and direction vectors
            self.thruster_positions[i, :] = np.array([params.get(f't{i + 1}_x') + params.get('xg'),
                                                    params.get(f't{i + 1}_y') + params.get('yg'),
                                                    params.get(f't{i + 1}_z') + params.get('zg')])
            direction_vector = np.array([params.get(f't{i + 1}_dir_x'),
                                        params.get(f't{i + 1}_dir_y'),
                                        params.get(f't{i + 1}_dir_z')])
            self.thruster_directions[i, :] = direction_vector / np.linalg.norm(direction_vector)

        # Load thruster data
        self.load_thruster_data()

    def calculate(self, state, pwm_array) -> np.ndarray:
        """
            Calculate the thrust forces based on PWM values.
            
            Input:
                pwm_array: Array of PWM values for the thrusters.
            Output:
                tau_t: External force and moment due to thrusters in body fixed frame, in (6,) format.
        """
        # Convert PWM values to thrust forces
        thrust_forces = self.pwm_to_force(state, pwm_array) * self.thruster_directions

        # Calculate thrust moments
        thrust_moments = np.cross(self.thruster_positions, thrust_forces)

        # Combine thrust forces and moments into a single vector
        tau_t = np.zeros((6,))
        tau_t[0:3] = np.sum(thrust_forces, axis=0).reshape((3,))
        tau_t[3:6] = np.sum(thrust_moments, axis=0).reshape((3,))

        return tau_t

    def load_thruster_data(self):
        """Load thruster data from the specified file under thruster data."""
        data = np.loadtxt(f"{self.params.params_folder}{self.params.thrusters.data}", delimiter=',', skiprows=1)

        # Extract the relevant columns
        self.voltage_values = np.unique(data[:, 0])  # Voltage is column A (0-indexed)
        self.pwm_values = np.unique(data[:, 1])      # PWM is column B (0-indexed)
        force = data[:, 6]                      # Force is column G (0-indexed)

        # Initialize force_values matrix
        num_rows = len(self.voltage_values)
        num_cols = len(self.pwm_values)
        self.force_values = np.zeros((num_rows, num_cols))

        # Fill in the matrix row by row
        for i in range(num_rows):
            start_idx = i * num_cols
            end_idx = (i + 1) * num_cols
            self.force_values[i, :] = force[start_idx:end_idx]

        # Convert from kgf to Newtons (1 kgf = 9.81 N)
        self.force_values *= 9.81

    def pwm_to_force(self, state, pwm_array) -> np.ndarray:
        """
            Convert PWM values to thrust forces using the thruster data.

            Input:
                pwm_array: Array of PWM values for the thrusters.
            Output:
                thrust_forces: The calculated thrust forces from each thruster.
        """
        # Unpack state
        voltage = unpack_state_object(state)[12]

        # Make sure pwm is in the range [1100, 1900]
        pwm_array = np.clip(pwm_array, 1100, 1900)

        # Create the interpolator
        interpolator = RegularGridInterpolator(
            (self.voltage_values, self.pwm_values),
            self.force_values,
            method='linear',
            bounds_error=False,
            fill_value=None
        )

        # Interpolate forces for each PWM value at the given voltage
        points = np.array([(voltage, pwm) for pwm in pwm_array])
        return interpolator(points)[:, np.newaxis]
