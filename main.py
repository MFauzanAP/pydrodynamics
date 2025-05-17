import time
import socket

from lib.vehicle import Vehicle
from lib.utils import unpack_state_object

TCP_IP = '127.0.0.1'
TCP_PORT = 5005
BUFFER_SIZE = 1024

TARGET_FPS = 10
TARGET_DT = 1 / TARGET_FPS

auv = Vehicle("examples/bpv2.yml")

def transform_pos_rot(state):
    x, y, z, phi, theta, psi = unpack_state_object(state)[0:6]
    x, y, z = x * 1, y * 1, z * -1
    return x, y, z, phi, theta, psi

def main_loop(dt):
    next_state = auv.step(dt, [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
    x, y, z, phi, theta, psi = transform_pos_rot(next_state)
    conn.sendall(f"{x},{y},{z},{phi},{theta},{psi}\n".encode())

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((TCP_IP, TCP_PORT))
    s.listen()
    print(f"Listening on {TCP_IP}:{TCP_PORT}")
    conn, addr = s.accept()
    with conn:
        print(f'Connection address: {addr}')
        while True:
            start_time = time.time()
            elapsed_time = time.time() - start_time

            # Main loop (use target_dt to make sure the loop runs at a fixed rate, not based on OS time)
            main_loop(TARGET_DT)

            # Make sure each iteration takes TARGET_DT seconds
            sleep_time = TARGET_DT - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)