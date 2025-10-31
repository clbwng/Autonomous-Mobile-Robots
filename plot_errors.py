
import matplotlib.pyplot as plt
from utilities import FileReader
import argparse
import math
import numpy as np

# ============================================================================
# CONFIGURATION SECTION - MODIFY THESE PATHS AND TARGET
# ============================================================================

# File paths for robot pose CSV files
P_CONTROLLER_POSE_FILE = "P_robot_pose_ns.csv"
PD_CONTROLLER_POSE_FILE = "PD_point_0.5klp_0.5klv_0.8kap_0.9kav/robot_pose.csv"

# Target point for point navigation [x, y]
TARGET_POINT = [1.0, 1.0]

# ============================================================================
# END CONFIGURATION
# ============================================================================
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
    axes[0].set_xlabel("X Position (m)")
    axes[0].set_ylabel("Y Position (m)")
    axes[0].grid()

    # --- START: Added Trajectory Plotting ---
    # If the file is 'robot_pose.csv', plot the target trajectory
    if 'robot_pose.csv' in filename:
        print(f"Plotting target trajectory for {filename}")
        pts = []
        # x_max = 2.5                                     # SIGMOID
        x_max = 1.5                                   # PARABOLA
        x = 0.0 # Assuming x starts at 0
        dx = 0.01 # Using a small step for a smooth curve (self.dx)
        while x < x_max + 1e-9:
            # y = 2.0 / (1.0 + math.exp(-2.0 * x)) - 1.0  # SIGMOID
            y = x * x                                 # PARABOLA
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

def calculate_robot_performance_metrics(pose_file, target_point=[1.0, 1.0]):
    """
    Calculate agility, accuracy, and overshoot metrics from robot pose data only
    
    Args:
        pose_file: Path to robot pose CSV file (x, y, th, stamp)
        target_point: Target position [x, y] for point navigation
        
    Returns:
        Dictionary with calculated metrics
    """
    
    # Load pose data
    try:
        pose_headers, pose_values = FileReader(pose_file).read_file()
    except Exception as e:
        print(f"Error loading pose file {pose_file}: {e}")
        return None
    
    if not pose_values:
        print(f"Error: No data found in {pose_file}")
        return None
    
    # Convert timestamps to seconds and extract pose data
    time = np.array([(val[-1] - pose_values[0][-1]) / 1e9 for val in pose_values])
    x_pos = np.array([val[0] for val in pose_values])
    y_pos = np.array([val[1] for val in pose_values])
    theta = np.array([val[2] for val in pose_values])
    
    # Calculate distance to target over time
    target_x, target_y = target_point
    distance_to_target = np.sqrt((x_pos - target_x)**2 + (y_pos - target_y)**2)
    target_distance = np.sqrt(target_x**2 + target_y**2)  # Distance from origin to target
    
    # Calculate angular error to target
    desired_angle = np.arctan2(target_y - y_pos, target_x - x_pos)
    angular_error = np.abs(desired_angle - theta)
    # Handle angle wraparound
    angular_error = np.minimum(angular_error, 2*np.pi - angular_error)
    
    metrics = {}
    
    print(f"Analyzing robot performance from {pose_file}...")
    
    # === AGILITY METRICS ===
    
    # 1. Rise Time - Time to reach 90% closer to target
    try:
        initial_distance = distance_to_target[0]
        final_distance = np.mean(distance_to_target[-20:])  # Average of last 20 points
        
        # 10% and 90% of the way to target
        distance_90_percent = initial_distance * 0.1 + final_distance * 0.9
        distance_10_percent = initial_distance * 0.9 + final_distance * 0.1
        
        idx_10 = np.where(distance_to_target <= distance_10_percent)[0]
        idx_90 = np.where(distance_to_target <= distance_90_percent)[0]
        
        if len(idx_10) > 0 and len(idx_90) > 0:
            rise_time = time[idx_90[0]] - time[idx_10[0]]
        else:
            rise_time = np.inf
            
        metrics['rise_time'] = rise_time
    except:
        metrics['rise_time'] = np.inf
    
    # 2. Settling Time - Time to stay within 5% of target
    try:
        tolerance = 0.05 * target_distance  # 5% of target distance (e.g., 5cm for 1m target)
        settled_indices = np.where(distance_to_target <= tolerance)[0]
        
        if len(settled_indices) > 0:
            settling_time = time[settled_indices[0]]
        else:
            settling_time = np.inf
            
        metrics['settling_time'] = settling_time
    except:
        metrics['settling_time'] = np.inf
    
    # === ACCURACY METRICS ===
    
    # 3. Steady-State Error - Final distance to target
    steady_state_error = np.mean(distance_to_target[-50:])  # Average of last 50 points
    metrics['steady_state_error'] = steady_state_error
    
    # 4. RMS Error - Root mean square of distance error over entire trajectory
    rms_error = np.sqrt(np.mean(distance_to_target**2))
    metrics['rms_error'] = rms_error
    
    # 5. Mean Absolute Error
    mae_error = np.mean(distance_to_target)
    metrics['mean_absolute_error'] = mae_error
    
    # === OVERSHOOT METRICS ===
    
    # 6. Position Overshoot - Did robot get closer than final position?
    try:
        min_distance = np.min(distance_to_target)
        final_distance = steady_state_error
        
        if min_distance < final_distance:
            overshoot = ((final_distance - min_distance) / target_distance) * 100
        else:
            overshoot = 0.0
            
        metrics['overshoot_percent'] = overshoot
        
        # Peak time (closest approach)
        peak_idx = np.argmin(distance_to_target)
        metrics['peak_time'] = time[peak_idx]
    except:
        metrics['overshoot_percent'] = 0.0
        metrics['peak_time'] = 0.0
    
    # === ADDITIONAL USEFUL METRICS ===
    
    # 7. Path Length - Total distance traveled
    path_segments = np.sqrt(np.diff(x_pos)**2 + np.diff(y_pos)**2)
    total_path_length = np.sum(path_segments)
    metrics['path_length'] = total_path_length
    
    # 8. Path Efficiency - Ratio of direct distance to actual path
    direct_distance = np.sqrt((x_pos[-1] - x_pos[0])**2 + (y_pos[-1] - y_pos[0])**2)
    if total_path_length > 0:
        path_efficiency = direct_distance / total_path_length
    else:
        path_efficiency = 0.0
    metrics['path_efficiency'] = path_efficiency
    
    # 9. Final Angular Error
    final_angular_error = np.mean(angular_error[-20:])
    metrics['final_angular_error'] = final_angular_error
    
    return metrics

def compare_robot_controllers(p_pose_file, pd_pose_file, target_point=[1.0, 1.0]):
    """
    Compare P and PD controllers using only robot pose data
    
    Args:
        p_pose_file: Path to P controller robot pose CSV file
        pd_pose_file: Path to PD controller robot pose CSV file
        target_point: Target position [x, y]
    """
    
    print("Calculating P Controller Performance...")
    p_metrics = calculate_robot_performance_metrics(p_pose_file, target_point)
    
    print("Calculating PD Controller Performance...")
    pd_metrics = calculate_robot_performance_metrics(pd_pose_file, target_point)
    
    if p_metrics is None or pd_metrics is None:
        print("Error: Could not calculate metrics for one or both controllers")
        return
    
    # Print comparison table
    print("\n" + "="*80)
    print("ROBOT PERFORMANCE COMPARISON (P vs PD Controllers)")
    print("="*80)
    print(f"Target Point: [{target_point[0]:.1f}, {target_point[1]:.1f}]")
    print("-"*80)
    print(f"{'Metric':<25} {'P Controller':<15} {'PD Controller':<15} {'Units':<15} {'Better':<10}")
    print("-"*80)
    
    # Helper function to determine better performance
    def better_performance(p_val, pd_val, lower_is_better=True):
        if lower_is_better:
            return "PD" if pd_val < p_val else "P"
        else:
            return "PD" if pd_val > p_val else "P"
    
    # AGILITY
    print("AGILITY:")
    rise_better = better_performance(p_metrics['rise_time'], pd_metrics['rise_time'])
    settle_better = better_performance(p_metrics['settling_time'], pd_metrics['settling_time'])
    
    print(f"{'  Rise Time':<25} {p_metrics['rise_time']:<15.3f} {pd_metrics['rise_time']:<15.3f} {'seconds':<15} {rise_better:<10}")
    print(f"{'  Settling Time':<25} {p_metrics['settling_time']:<15.3f} {pd_metrics['settling_time']:<15.3f} {'seconds':<15} {settle_better:<10}")
    
    # ACCURACY  
    print("\nACCURACY:")
    steady_better = better_performance(p_metrics['steady_state_error'], pd_metrics['steady_state_error'])
    rms_better = better_performance(p_metrics['rms_error'], pd_metrics['rms_error'])
    mae_better = better_performance(p_metrics['mean_absolute_error'], pd_metrics['mean_absolute_error'])
    
    print(f"{'  Steady-State Error':<25} {p_metrics['steady_state_error']:<15.3f} {pd_metrics['steady_state_error']:<15.3f} {'meters':<15} {steady_better:<10}")
    print(f"{'  RMS Error':<25} {p_metrics['rms_error']:<15.3f} {pd_metrics['rms_error']:<15.3f} {'meters':<15} {rms_better:<10}")
    print(f"{'  Mean Absolute Error':<25} {p_metrics['mean_absolute_error']:<15.3f} {pd_metrics['mean_absolute_error']:<15.3f} {'meters':<15} {mae_better:<10}")
    
    # OVERSHOOT
    print("\nOVERSHOOT:")
    overshoot_better = better_performance(p_metrics['overshoot_percent'], pd_metrics['overshoot_percent'])
    
    print(f"{'  Position Overshoot':<25} {p_metrics['overshoot_percent']:<15.3f} {pd_metrics['overshoot_percent']:<15.3f} {'percent':<15} {overshoot_better:<10}")
    print(f"{'  Peak Time':<25} {p_metrics['peak_time']:<15.3f} {pd_metrics['peak_time']:<15.3f} {'seconds':<15} {'-':<10}")
    
    # ADDITIONAL METRICS
    print("\nADDITIONAL METRICS:")
    path_better = better_performance(p_metrics['path_length'], pd_metrics['path_length'])
    efficiency_better = better_performance(p_metrics['path_efficiency'], pd_metrics['path_efficiency'], lower_is_better=False)
    angular_better = better_performance(p_metrics['final_angular_error'], pd_metrics['final_angular_error'])
    
    print(f"{'  Path Length':<25} {p_metrics['path_length']:<15.3f} {pd_metrics['path_length']:<15.3f} {'meters':<15} {path_better:<10}")
    print(f"{'  Path Efficiency':<25} {p_metrics['path_efficiency']:<15.3f} {pd_metrics['path_efficiency']:<15.3f} {'ratio':<15} {efficiency_better:<10}")
    print(f"{'  Final Angular Error':<25} {p_metrics['final_angular_error']:<15.3f} {pd_metrics['final_angular_error']:<15.3f} {'radians':<15} {angular_better:<10}")
    
    print("\n" + "="*80)
    
    # OVERALL SUMMARY
    print("OVERALL PERFORMANCE SUMMARY:")
    
    # Count wins for each controller
    p_wins = 0
    pd_wins = 0
    
    key_metrics = [
        ('settling_time', True), ('steady_state_error', True), ('rms_error', True), 
        ('overshoot_percent', True), ('path_efficiency', False)
    ]
    
    for metric, lower_is_better in key_metrics:
        if lower_is_better:
            if pd_metrics[metric] < p_metrics[metric]:
                pd_wins += 1
            else:
                p_wins += 1
        else:
            if pd_metrics[metric] > p_metrics[metric]:
                pd_wins += 1
            else:
                p_wins += 1
    
    if pd_wins > p_wins:
        print("🏆 PD Controller shows BETTER overall performance")
        print(f"   PD wins in {pd_wins}/{len(key_metrics)} key metrics")
    elif p_wins > pd_wins:
        print("🏆 P Controller shows BETTER overall performance") 
        print(f"   P wins in {p_wins}/{len(key_metrics)} key metrics")
    else:
        print("🤝 Controllers show COMPARABLE performance")
        print(f"   Tied performance in key metrics")
    
    print("="*80)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process robot data files.')
    parser.add_argument('--files', nargs='+', help='List of files to process')
    parser.add_argument('--compare-controllers', action='store_true', help='Compare P and PD controllers using configured file paths')
    
    # Optional: Override configured file paths
    parser.add_argument('--p-pose', type=str, help='P controller robot pose CSV file (overrides configured path)')
    parser.add_argument('--pd-pose', type=str, help='PD controller robot pose CSV file (overrides configured path)')
    
    # Optional: Override configured target point
    parser.add_argument('--target', nargs=2, type=float, help='Target point [x y] (overrides configured target)')
    
    args = parser.parse_args()
    
    if args.compare_controllers:
        # Use configured file paths unless overridden by command line
        p_pose_file = args.p_pose if args.p_pose else P_CONTROLLER_POSE_FILE
        pd_pose_file = args.pd_pose if args.pd_pose else PD_CONTROLLER_POSE_FILE
        target_point = args.target if args.target else TARGET_POINT
        
        print("="*60)
        print("ROBOT CONTROLLER COMPARISON")
        print("="*60)
        print(f"P Controller file:  {p_pose_file}")
        print(f"PD Controller file: {pd_pose_file}")
        print(f"Target point:       {target_point}")
        print("="*60)
        
        compare_robot_controllers(p_pose_file, pd_pose_file, target_point)
    
    elif args.files:
        print("Plotting the files", args.files)
        filenames = args.files
        for filename in filenames:
            plot_errors(filename)
    
    else:
        print("Please specify either --files for plotting or --compare-controllers for metrics analysis")
        print("\nQuick usage:")
        print("  python3 plot_errors.py --compare-controllers")
        print("  (uses configured file paths at top of script)")
        print()
        parser.print_help()