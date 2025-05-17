import math
import numpy as np

from dataclasses import dataclass

@dataclass
class Position:
    x: float # m
    y: float # m
    z: float # m

@dataclass
class Orientation:
    roll: float # degrees
    pitch: float # degrees
    yaw: float # degrees

@dataclass
class LinearVelocity:
    u: float # m/s
    v: float # m/s
    w: float # m/s

@dataclass
class AngularVelocity:
    p: float # deg/s
    q: float # deg/s
    r: float # deg/s

@dataclass
class State:
    position: Position
    orientation: Orientation
    linear_velocity: LinearVelocity
    angular_velocity: AngularVelocity
    voltage: float

@dataclass
class EnvironmentParams:
    gravity: float = 9.81 # m/s^2
    density: float = 1000 # kg/m^3

@dataclass
class PhysicalParams:
    mass: float # kg
    volume: float # m^3

    com: Position # Center of mass
    cob : Position # Center of buoyancy

    inertia: Position # Inertia tensor (Ixx, Iyy, Izz)

@dataclass
class ElectricalParams:
    voltage: float # V
    capacity: float # mAh

@dataclass
class ThrusterData:
    name: str
    pos: Position # Thruster position
    dir: Position # Thruster direction

@dataclass
class ThrusterParams:
    data: str
    list: list[ThrusterData]

@dataclass
class Params:
    name: str
    verbose: bool
    env: EnvironmentParams
    physical: PhysicalParams
    electrical: ElectricalParams
    thrusters: ThrusterParams

def body_to_world(state: np.ndarray) -> np.ndarray:
    """
        Convert body-fixed velocities to world frame.
        Input:
            state: State array containing orientation and body-fixed velocities.
        Output:
            world_velocities: World frame velocities.
    """
    # Unpack state
    roll, pitch, yaw = state[3], state[4], state[5]
    u, v, w = state[6], state[7], state[8]
    p, q, r = state[9], state[10], state[11]

    # Convert angles to radians
    roll, pitch, yaw = np.radians([roll, pitch, yaw])

    # Rotation matrix from body-fixed frame to world frame
    c_roll = math.cos(roll)
    c_pitch = math.cos(pitch)
    c_yaw = math.cos(yaw)
    s_roll = math.sin(roll)
    s_pitch = math.sin(pitch)
    s_yaw = math.sin(yaw)
    t_pitch = math.tan(pitch)
    r_trans = np.array([
        [c_yaw*c_pitch, -s_yaw*c_roll + c_yaw*s_pitch*s_roll, s_yaw*s_roll + c_yaw*s_pitch*c_roll],
        [s_yaw*c_pitch, c_yaw*c_roll + s_yaw*s_pitch*s_roll, -c_yaw*s_roll + s_yaw*s_pitch*c_roll],
        [-s_pitch, c_pitch*s_roll, c_pitch*c_roll]
    ])
    r_rot = np.array([
        [1, s_roll*t_pitch, c_roll*t_pitch],
        [0, c_roll, -s_roll],
        [0, s_roll/c_pitch, c_roll/c_pitch]
    ])

    # Convert from body-fixed to world frame
    return np.concatenate((
        np.matmul(r_trans, np.array([u, v, w])),
        np.matmul(r_rot, np.array([p, q, r]))
    ), axis=0).reshape((6,))

def world_to_body(state: np.ndarray) -> np.ndarray:
    """
        Convert world frame velocities to body-fixed frame.
        Input:
            state: State array containing orientation and world frame velocities.
        Output:
            body_velocities: Body-fixed velocities.
    """
    # Unpack state
    roll, pitch, yaw = state[3], state[4], state[5]
    u, v, w = state[6], state[7], state[8]
    p, q, r = state[9], state[10], state[11]

    # Convert angles to radians
    roll, pitch, yaw = np.radians([roll, pitch, yaw])

    # Rotation matrix from body-fixed frame to world frame
    c_roll = math.cos(roll)
    c_pitch = math.cos(pitch)
    c_yaw = math.cos(yaw)
    s_roll = math.sin(roll)
    s_pitch = math.sin(pitch)
    s_yaw = math.sin(yaw)
    t_pitch = math.tan(pitch)
    r_trans = np.array([
        [c_yaw*c_pitch, -s_yaw*c_roll + c_yaw*s_pitch*s_roll, s_yaw*s_roll + c_yaw*s_pitch*c_roll],
        [s_yaw*c_pitch, c_yaw*c_roll + s_yaw*s_pitch*s_roll, -c_yaw*s_roll + s_yaw*s_pitch*c_roll],
        [-s_pitch, c_pitch*s_roll, c_pitch*c_roll]
    ])
    r_rot = np.array([
        [1, s_roll*t_pitch, c_roll*t_pitch],
        [0, c_roll, -s_roll],
        [0, s_roll/c_pitch, c_roll/c_pitch]
    ])

    # Inverse rotation matrix from world frame to body-fixed frame
    inv_r_trans = np.linalg.inv(r_trans)
    inv_r_rot = np.linalg.inv(r_rot)

    # Convert world to body-fixed frame
    return np.concatenate((
        np.matmul(inv_r_trans, np.array([u, v, w])),
        np.matmul(inv_r_rot, np.array([p, q, r]))
    ), axis=0).reshape((6,))

def fix_angular_circularity(state: np.ndarray) -> np.ndarray:
    """
        Ensure that the angular velocities are within the range of -180 to 180 degrees.
        Input:
            state: State array containing orientation and angular velocities.
        Output:
            state: Updated state with fixed angular velocities.
    """
    state[3] = (state[3] + 180) % 360 - 180
    state[4] = (state[4] + 180) % 360 - 180
    state[5] = (state[5] + 180) % 360 - 180
    return state
