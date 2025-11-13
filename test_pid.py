import ctypes
import json

DRONE_MASS = 1.0
DELTA_TIME = 0.05
SIMULATION_STEPS = 1000

MAX_ANGLE_DEGREES = 25.0
THRUST_FACTOR = 0.1
DRAG_FACTOR = 0.1

START_POSITION = [-100.0, 50.0]
TARGET_POSITION = [0.0, 0.0]

try:
    pid_lib = ctypes.CDLL('./obj/pid.so')
except OSError as e:
    print(f"Error loading library: {e}")
    print("Make sure './obj/pid.so' exists and your C++ code compiled.")
    exit()


pid_lib.create_pid.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
pid_lib.create_pid.restype = ctypes.c_void_p

pid_lib.calculate_pid.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_uint32)
]
pid_lib.calculate_pid.restype = None

pid_lib.destroy_pid.argtypes = [ctypes.c_void_p]
pid_lib.destroy_pid.restype = None

FloatArray2 = ctypes.c_float * 2
UInt32Array2 = ctypes.c_uint32 * 2

def map_value(x, in_min, in_max, out_min, out_max):
    """Linearly map a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def run_simulation():
    print("Python: Creating PID controller...")
    pid_handle = pid_lib.create_pid(-10.0, 1.0, 1.0, 0.0)
    if not pid_handle:
        print("Error: create_pid returned NULL. Check C++ constructor.")
        return

    current_position = list(START_POSITION)
    current_velocity = [0.0, 0.0]

    path_history = {
        "target": TARGET_POSITION,
        "start": START_POSITION,
        "path": []
    }

    c_target_pos = FloatArray2(*TARGET_POSITION)
    c_current_pos = FloatArray2(*current_position)
    c_angle_commands = UInt32Array2()

    print("Python: Running simulation...")
    for step in range(SIMULATION_STEPS):
        path_history["path"].append(tuple(current_position))

        c_current_pos[0] = current_position[0]
        c_current_pos[1] = current_position[1]
        pid_lib.calculate_pid(pid_handle, c_current_pos, c_target_pos, c_angle_commands)
        for i in range(2):
            pwm_command = c_angle_commands[i]
            angle_deg = map_value(pwm_command, 1000, 2000, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
            
            force = angle_deg * THRUST_FACTOR

            drag_force = -DRAG_FACTOR * current_velocity[i]

            net_force = force + drag_force
            acceleration = net_force / DRONE_MASS

            current_velocity[i] += acceleration * DELTA_TIME
            current_position[i] += current_velocity[i] * DELTA_TIME

    print("Python: Simulation finished.")

    pid_lib.destroy_pid(pid_handle)

    output_filename = "src/pid/path_data.json"
    with open(output_filename, 'w') as f:
        json.dump(path_history, f, indent=2)
    print(f"Python: Path data saved to {output_filename}")

if __name__ == "__main__":
    run_simulation()
    