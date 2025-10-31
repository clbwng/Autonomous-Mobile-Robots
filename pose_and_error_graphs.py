
# import matplotlib.pyplot as plt
# from utilities import FileReader
# import argparse
# import math
# def plot_errors(filename):
#     headers, values = FileReader(filename).read_file()
#     time_list = []
#     # Check if values list is empty to avoid index error
#     if not values:
#         print(f"No data found in {filename}")
#         return
#     first_stamp = values[0][-1]
#     for val in values:
#         time_list.append(val[-1] - first_stamp)

#     fig, axes = plt.subplots(1, 2, figsize=(14, 6))

#     # --- State Space Plot (axes[0]) ---
#     # Plot the data from the file
#     axes[0].plot([lin[0] for lin in values], [lin[1] for lin in values], label="Actual Pose")
#     axes[0].set_title("State Space")
#     axes[0].grid()

#     # --- START: Added Trajectory Plotting ---
#     # If the file is 'robot_pose.csv', plot the target trajectory
#     if 'robot_pose.csv' in filename:
#         print(f"Plotting target trajectory for {filename}")
#         pts = []
#         # x_max = 2.5 # SIGMOID
#         x_max = 1.5 # PARABOLA
#         x = 0.0 # Assuming x starts at 0
#         dx = 0.01 # Using a small step for a smooth curve (self.dx)
#         while x < x_max + 1e-9:
#             # y = 2.0 / (1.0 + math.exp(-2.0 * x)) - 1.0 # SIGMOID
#             y = x * x # PARABOLA
#             pts.append([x, y])
#             x += dx
#         # Extract x and y columns for plotting
#         x_traj = [p[0] for p in pts]
#         y_traj = [p[1] for p in pts]
#         # Plot the target trajectory on the state space plot
#         axes[0].plot(x_traj, y_traj,
#             label="Target Trajectory",
#             linestyle='--',
#             color='red')
#         axes[0].set_xlabel("x [m]"); axes[0].set_ylabel("y [m]")


#     # Add legend to the state space plot
#     axes[0].legend()
#     # --- END: Added Trajectory Plotting ---

#     # --- Individual States Plot (axes[1]) ---
#     axes[1].set_title("Each Individual State")
#     for i in range(0, len(headers) - 1):
#         axes[1].plot(time_list, [lin[i] for lin in values], label=headers[i] + " linear")
    
#     axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("state value [units]")
#     #axes[1].plot(time_list, series, label=..., marker='o', markevery=max(1, len(time_list)//25))

#     axes[1].legend()
#     axes[1].grid()

#     plt.show()

# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description='Process some files.')
#     parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
#     args = parser.parse_args()
#     print("Plotting the files", args.files)

#     filenames = args.files
#     for filename in filenames:
        # plot_errors(filename)
import matplotlib.pyplot as plt
from utilities import FileReader
import argparse
import math

def _infer_time_scale(stamps):
    # Infer units from typical step between samples
    if len(stamps) < 2:
        return 1.0
    dts = [stamps[i+1] - stamps[i] for i in range(len(stamps)-1)]
    med_dt = sorted(dts)[len(dts)//2]

    # Rough bands: ns >> us >> ms >> s
    if med_dt > 1e5:      # e.g., 10 ms = 1e7 ns
        return 1e-9       # ns -> s
    elif med_dt > 1e2:    # e.g., 10 ms = 1e4 us
        return 1e-6       # µs -> s
    elif med_dt > 1e-1:   # e.g., 10 ms = 10 ms
        return 1e-3       # ms -> s
    else:
        return 1.0        # already in seconds

def _series_label_from_filename(fname):
    up = fname.upper()
    if "PID" in up:
        return "PID"
    if "PD" in up:
        return "PD"
    return "P"

def _units_for_header(h, is_pose=False):
    h = h.lower()
    if is_pose:
        if h in ("x", "y"): return "[m]"
        if h in ("th", "theta"): return "[rad]"
        return ""  # unknown
    # linear/angular logs
    if h in ("e", "eth", "e_th", "eang", "e_theta"):           # error
        return "[m]" if "ang" not in h and "th" not in h else "[rad]"
    if h in ("e_dot", "edot", "edot_th", "eth_dot"):            # derivative
        return "[m/s]" if "th" not in h else "[rad/s]"
    if h in ("e_int", "eint", "int_e"):                         # integral
        return "[m·s]"  if "th" not in h else "[rad·s]"
    return ""  # other fields (v, w, etc.) keep generic

def plot_errors(filename, axes=None, is_pose=False):
    headers, values = FileReader(filename).read_file()
    if not values:
        print(f"No data found in {filename}")
        return axes  # return whatever we got

    # Create axes (so multiple files overlay on the same figure)
    if axes is None:
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        if is_pose:
            # robot pose figure
            axes[0].set_title("State Space (x–y) — P vs PD")
            axes[0].set_xlabel("x [m]"); axes[0].set_ylabel("y [m]")
            axes[0].grid(); axes[0].set_aspect('equal', adjustable='box')
            axes[1].set_title("Pose States vs Time — P vs PD")
            axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("state value [m, rad]")
            axes[1].grid()
        else:
            # linear + angular figure
            axes[0].set_title("Phase / State-Space — P vs PD")
            axes[0].set_xlabel("state 0"); axes[0].set_ylabel("state 1")
            axes[0].grid()
            axes[1].set_title("States vs Time — P vs PD")
            axes[1].set_xlabel("time [s]"); axes[1].set_ylabel("state value")
            axes[1].grid()

    # Build time list (relative, in seconds)
    stamps = [row[-1] for row in values]

    tscale = 1e-9      
    t0 = stamps[0]
    time_s = [(s - t0) * tscale for s in stamps]

    # Decide legend controller label from filename
    series_label = _series_label_from_filename(filename)

    # --- LEFT subplot ---
    # For robot_pose: use first two columns as x,y
    # For linear/angular: if e & e_dot exist, prefer plotting (e vs e_dot); else col0 vs col1.
    lower_headers = [h.lower() for h in headers]
    idx = {h.lower(): i for i, h in enumerate(headers)}

    if is_pose:
        x_idx = 0
        y_idx = 1 if len(headers) > 1 else 0
        axes[0].plot(
            [row[x_idx] for row in values],
            [row[y_idx] for row in values],
            label=f"{series_label} Pose"
        )
        # Optional target trajectory: uncomment to show
        # if 'robot_pose.csv' in filename:
        #     pts = []
        #     x_max = 1.5  # PARABOLA (or use 2.5 for SIGMOID)
        #     x = 0.0; dx = 0.01
        #     while x < x_max + 1e-9:
        #         # y = 2.0 / (1.0 + math.exp(-2.0 * x)) - 1.0  # SIGMOID
        #         y = x * x  # PARABOLA
        #         pts.append((x, y)); x += dx
        #     axes[0].plot([p[0] for p in pts], [p[1] for p in pts],
        #                  label="Target Trajectory", linestyle='--')
    else:
        # linear/angular: try to be smart about e vs e_dot
        if ("e" in idx) and (("e_dot" in idx) or ("edot" in idx)):
            e_idx = idx["e"]
            edot_idx = idx["e_dot"] if "e_dot" in idx else idx["edot"]
            axes[0].plot(
                [row[e_idx] for row in values],
                [row[edot_idx] for row in values],
                label=f"{series_label} (e–ė)"
            )
            axes[0].set_xlabel(f"e { _units_for_header('e') }")
            axes[0].set_ylabel(f"ė { _units_for_header('e_dot') }")
        else:
            # fallback: first two columns as a generic state-space view
            axes[0].plot(
                [row[0] for row in values],
                [row[1] for row in values],
                label=f"{series_label} (col0–col1)"
            )
    axes[0].legend()

    # --- RIGHT subplot: every column vs time (except stamp) ---
    SKIP_COLUMNS = {"e_int", "eint", "int_e"}  # lowercase
    # Include unit per series in the legend label
    for i in range(0, len(headers) - 1):
        name = headers[i]
        if name.strip().lower() in SKIP_COLUMNS:
            continue  # don't plot e_int
        unit = _units_for_header(name, is_pose=is_pose)
        axes[1].plot(
            time_s,
            [row[i] for row in values],
            label=f"{series_label}: {name} {unit}"
        )
    # Y-label chosen earlier; legend covers per-series units
    axes[1].legend()

    return axes


# TRACJECTORIES
import math

def _make_parabola_pts(x_max=1.5, dx=0.01):
    xs, ys = [], []
    x = 0.0
    while x <= x_max + 1e-12:
        xs.append(x); ys.append(x*x)
        x += dx
    return xs, ys

def _make_sigmoid_pts(x_max=2.5, dx=0.01):
    xs, ys = [], []
    x = 0.0
    while x <= x_max + 1e-12:
        y = 2.0 / (1.0 + math.exp(-2.0 * x)) - 1.0
        xs.append(x); ys.append(y)
        x += dx
    return xs, ys

def plot_final_xy(parabola_pose_csv, sigmoid_pose_csv, show_targets=True, label_prefix="PD "):
    """
    Make ONE figure with x–y for the final chosen controller on:
      - Parabola trajectory
      - Sigmoid trajectory
    Optionally overlays the ideal target curves (dashed).
    """
    # Load pose logs: expect columns [x, y, ..., stamp] like your robot_pose.csv
    headers_p, vals_p = FileReader(parabola_pose_csv).read_file()
    headers_s, vals_s = FileReader(sigmoid_pose_csv).read_file()
    if not vals_p or not vals_s:
        print("Missing data for final trajectories."); return

    xpi, ypi = 0, 1  # first two columns are x,y in your pose logs
    xsi, ysi = 0, 1

    fig, ax = plt.subplots(figsize=(6.5, 6.5))

    # Actual tracks
    ax.plot([r[xpi] for r in vals_p], [r[ypi] for r in vals_p],
            label=f"Parabola - Experimental", marker='', linestyle='-', markevery=max(1, len(vals_p)//30))
    ax.plot([r[xsi] for r in vals_s], [r[ysi] for r in vals_s],
            label=f"Sigmoid - Experimental", marker='', linestyle='-', markevery=max(1, len(vals_s)//30))

    # Optional ideal curves
    if show_targets:
        px, py = _make_parabola_pts()
        sx, sy = _make_sigmoid_pts()
        ax.plot(px, py, '--', label="Parabola - Target")
        ax.plot(sx, sy, '--', label="Sigmoid - Target")

    ax.set_title("PD Controller — Trajectories (x–y)")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.grid(True); ax.set_aspect('equal', adjustable='box')
    ax.legend()
    fig.tight_layout()
    fig.savefig("final_controller_xy_parabola_sigmoid.pdf", bbox_inches='tight', dpi=300)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    args = parser.parse_args()
    print("Plotting the files", args.files)

    axes_dyn = None   # one figure for linear + angular
    axes_pose = None  # another figure for robot_pose

    # for filename in args.files:
    #     lower = filename.lower()
    #     if 'robot_pose' in lower:
    #         axes_pose = plot_errors(filename, axes=axes_pose, is_pose=True)
    #     elif ('linear' in lower) or ('ang' in lower) or ('angular' in lower):
    #         axes_dyn = plot_errors(filename, axes=axes_dyn, is_pose=False)
    #     else:
    #         # default to dyn fig if unknown
    #         axes_dyn = plot_errors(filename, axes=axes_dyn, is_pose=False)
    
    plot_final_xy(args.files[0], args.files[1])
    plt.show()
