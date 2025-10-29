
import matplotlib.pyplot as plt
from utilities import FileReader
import argparse
import math
def plot_errors(filename):
    headers, values = FileReader(filename).read_file()
    time_list = []
    # Check if values list is empty to avoid index error
    if not values:
        print(f"No data found in {filename}")
        return
    first_stamp = values[0][-1]
    for val in values:
        time_list.append(val[-1] - first_stamp)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    # --- State Space Plot (axes[0]) ---
    # Plot the data from the file
    axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values], label="Actual Pose")
    axes[0].set_title("State Space")
    axes[0].grid()

    # --- START: Added Trajectory Plotting ---
    # If the file is 'robot_pose.csv', plot the target trajectory
    if 'robot_pose.csv' in filename:
        print(f"Plotting target trajectory for {filename}")
        pts = []
        x_max = 2.5                                     # SIGMOID
        # x_max = 1.5                                   # PARABOLA
        x = 0.0 # Assuming x starts at 0
        dx = 0.01 # Using a small step for a smooth curve (self.dx)
        while x < x_max + 1e-9:
            y = 2.0 / (1.0 + math.exp(-2.0 * x)) - 1.0  # SIGMOID
            # y = x * x                                 # PARABOLA
            pts.append([x, y])
            x += dx
        # Extract x and y columns for plotting
        x_traj = [p[0] for p in pts]
        y_traj = [p[1] for p in pts]
        # Plot the target trajectory on the state space plot
        axes[0].plot(x_traj, y_traj,
            label="Target Trajectory",
            linestyle='--',
            color='red')

    # Add legend to the state space plot
    axes[0].legend()
    # --- END: Added Trajectory Plotting ---

    # --- Individual States Plot (axes[1]) ---
    axes[1].set_title("Each Individual State")
    for i in range(0, len(headers) - 1):
        axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i] + " linear")

    axes[1].legend()
    axes[1].grid()

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    args = parser.parse_args()
    print("Plotting the files", args.files)

    filenames = args.files
    for filename in filenames:
        plot_errors(filename)