import json
import matplotlib.pyplot as plt

def plot_data():
    input_filename = "src/pid/path_data.json"

    try:
        with open(input_filename, 'r') as f:
            data = json.load(f)
    except FileNotFoundError:
        print(f"Error: Could not find {input_filename}")
        print("Did you run the Docker container first?")
        return

    # Unzip the path data
    path = data['path']
    x_coords = [pos[0] for pos in path]
    y_coords = [pos[1] for pos in path]

    start_pos = data['start']
    target_pos = data['target']

    # --- Plotting ---
    plt.figure(figsize=(10, 8))

    # Plot the drone's path
    plt.plot(x_coords, y_coords, 'b-', label='Drone Path')

    # Plot start and end points
    plt.plot(start_pos[0], start_pos[1], 'go', markersize=10, label='Start')
    plt.plot(target_pos[0], target_pos[1], 'rx', markersize=15, label='Target')

    plt.title('Drone PID Simulation')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.grid(True)
    plt.axis('equal') # Ensure X and Y axes have the same scale

    # Save the plot to a file and show it
    plt.savefig('drone_path.png')
    print("Plot saved to drone_path.png")
    plt.show()

if __name__ == "__main__":
    plot_data()