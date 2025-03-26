import numpy as np
import matplotlib.pyplot as plt

def set_axes_equal(ax):
    """Sets equal scaling for all axes in a 3D plot."""
    x_limits = ax.get_xlim()
    y_limits = ax.get_ylim()
    z_limits = ax.get_zlim()

    x_range = x_limits[1] - x_limits[0]
    y_range = y_limits[1] - y_limits[0]
    z_range = z_limits[1] - z_limits[0]

    max_range = max(x_range, y_range, z_range) / 2.0

    x_middle = (x_limits[0] + x_limits[1]) / 2.0
    y_middle = (y_limits[0] + y_limits[1]) / 2.0
    z_middle = (z_limits[0] + z_limits[1]) / 2.0

    ax.set_xlim([x_middle - max_range, x_middle + max_range])
    ax.set_ylim([y_middle - max_range, y_middle + max_range])
    ax.set_zlim([z_middle - max_range, z_middle + max_range])

def plot_waypoints(waypoints, segment_types, ax):
    """Plot the generated 3D path with blue for lines and yellow for arcs."""
    
    start_idx = 0
    for segment_type, num_points in segment_types:
        x_vals = [p["position"][0] for p in waypoints[start_idx:start_idx + num_points]]
        y_vals = [p["position"][1] for p in waypoints[start_idx:start_idx + num_points]]
        z_vals = [p["position"][2] for p in waypoints[start_idx:start_idx + num_points]]
        
        # color = 'b' if segment_type == "line" else 'y'  # Blue for line, Yellow for arc
        if segment_type == "line" or segment_type == "S":
            color = 'b'
        else:
            color = 'y'
        ax.plot(x_vals, y_vals, z_vals, marker='o', linestyle='-', markersize=3, color=color, label=f"{segment_type} segment")

        start_idx += num_points

    # Start and End markers
    ax.scatter(waypoints[0]["position"][0], waypoints[0]["position"][1], waypoints[0]["position"][2], 
               color='b', marker='o', s=100, label="Start")
    ax.scatter(waypoints[-1]["position"][0], waypoints[-1]["position"][1], waypoints[-1]["position"][2], 
               color='b', marker='x', s=100, label="End")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("AUV 3D Path Visualization")
    ax.legend()

    