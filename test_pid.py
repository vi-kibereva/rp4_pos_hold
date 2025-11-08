import ctypes
import json

# --- Constants for our simulation ---
DRONE_MASS = 1.0           # (kg)
DELTA_TIME = 0.05          # (seconds) simulation time step
SIMULATION_STEPS = 1000

# --- Drone & PID Tuning ---
# New constants for angle mode
MAX_ANGLE_DEGREES = 25.0   # Max tilt angle (e.g., 25 degrees)
THRUST_FACTOR = 0.1        # Converts degrees of tilt to Force (N). TUNE THIS.
DRAG_FACTOR = 0.1          # Air resistance. TUNE THIS.

# --- Target and Start ---
START_POSITION = [-100.0, 50.0]
TARGET_POSITION = [0.0, 0.0]

# --- Load and configure the ARM PID library ---
# !!! IMPORTANT: Update this path to where your .so file is !!!
# The Dockerfile copies it to /app/pid.so, but the -v mount
# makes it available at its original location.
try:
    pid_lib = ctypes.CDLL('./obj/pid.so')
except OSError as e:
    print(f"Error loading library: {e}")
    print("Make sure './obj/pid.so' exists and your C++ code compiled.")
    exit()


# 1. void* create_pid(float, float, float, float)
pid_lib.create_pid.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]
pid_lib.create_pid.restype = ctypes.c_void_p

# 2. void calculate_pid(void* handle, float* desired, float* current, uint32_t* result_out)
pid_lib.calculate_pid.argtypes = [
    ctypes.c_void_p,
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_float),
    ctypes.POINTER(ctypes.c_uint32)
]
pid_lib.calculate_pid.restype = None

# 3. void destroy_pid(void* pid_handle)
pid_lib.destroy_pid.argtypes = [ctypes.c_void_p]
pid_lib.destroy_pid.restype = None

# --- C-style array types ---
FloatArray2 = ctypes.c_float * 2
UInt32Array2 = ctypes.c_uint32 * 2

# --- Helper function ---
def map_value(x, in_min, in_max, out_min, out_max):
    """Linearly map a value from one range to another."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# --- Simulation main function ---
def run_simulation():
    print("Python: Creating PID controller...")
    # Tune your Kp, Ki, Kd, Kdf values here
    pid_handle = pid_lib.create_pid(-10.0, 1.0, 1.0, 0.0)
    if not pid_handle:
        print("Error: create_pid returned NULL. Check C++ constructor.")
        return

    # --- Drone's physics state ---
    current_position = list(START_POSITION)
    current_velocity = [0.0, 0.0]

    # --- Data for plotting ---
    path_history = {
        "target": TARGET_POSITION,
        "start": START_POSITION,
        "path": []
    }

    # --- Ctypes arrays to pass to C++ ---
    c_target_pos = FloatArray2(*TARGET_POSITION)
    c_current_pos = FloatArray2(*current_position)
    c_angle_commands = UInt32Array2() # This is the output array (1000-2000)

    print("Python: Running simulation...")
    for step in range(SIMULATION_STEPS):
        path_history["path"].append(tuple(current_position))

        # 1. Call PID controller
        c_current_pos[0] = current_position[0]
        c_current_pos[1] = current_position[1]
        pid_lib.calculate_pid(pid_handle, c_current_pos, c_target_pos, c_angle_commands)
        # 2. Simulate Physics (The "Body")
        for i in range(2): # For Pitch (X axis) and Roll (Y axis)

            # --- THIS IS THE NEW LOGIC ---
            # A. Convert PID output (1000-2000) to an angle in degrees
            pwm_command = c_angle_commands[i]
            angle_deg = map_value(pwm_command, 1000, 2000, -MAX_ANGLE_DEGREES, MAX_ANGLE_DEGREES)

            # B. Convert angle to horizontal force
            #    This is a simple model; Force is proportional to tilt angle
            force = angle_deg * THRUST_FACTOR

            # C. Apply air resistance (drag)
            drag_force = -DRAG_FACTOR * current_velocity[i]

            # D. Newton's Law: F_net = m * a  =>  a = F_net / m
            net_force = force + drag_force
            acceleration = net_force / DRONE_MASS

            # E. Update velocity and position (Integrate)
            current_velocity[i] += acceleration * DELTA_TIME
            current_position[i] += current_velocity[i] * DELTA_TIME
            # --- END OF NEW LOGIC ---

    print("Python: Simulation finished.")

    # --- Clean up C++ object ---
    pid_lib.destroy_pid(pid_handle)

    # --- Save results to a file ---
    # Save to the correct location as requested
    output_filename = "src/pid/path_data.json"
    with open(output_filename, 'w') as f:
        json.dump(path_history, f, indent=2)
    print(f"Python: Path data saved to {output_filename}")

# --- Run the main function ---
if __name__ == "__main__":
    run_simulation()