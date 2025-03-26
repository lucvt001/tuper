import yaml
import numpy as np
import matplotlib.pyplot as plt
import csv
import rclpy
from rclpy.node import Node
from .utils import plot_waypoints, set_axes_equal
from scipy.spatial.transform import Rotation as R


def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def compute_orientation(prev_pos, curr_pos):
    """
    Computes the quaternion (qw, qx, qy, qz) representing the orientation
    given the direction from previous to current position.
    """
    direction = np.array(curr_pos) - np.array(prev_pos)
    direction /= np.linalg.norm(direction)  # Normalize to unit vector
    
    # Compute yaw (rotation around Z) from the XY plane
    yaw = np.arctan2(direction[1], direction[0])
    
    # Assume no roll/pitch (flat motion), create quaternion
    quat = R.from_euler('z', yaw).as_quat()  # Returns (qx, qy, qz, qw)
    
    # Convert to (qw, qx, qy, qz) for ROS compatibility
    return round(quat[3], 6), round(quat[0], 6), round(quat[1], 6), round(quat[2], 6)


def generate_line(start, end, velocity, dt, start_time):
    """Generate waypoints along a straight 3D line with velocity-based spacing."""
    start = np.array(start)
    end = np.array(end)
    direction = (end - start) / np.linalg.norm(end - start)
    total_distance = np.linalg.norm(end - start)
    
    step_size = velocity * dt
    num_steps = int(total_distance / step_size)

    waypoints = []
    for i in range(num_steps + 1):
        point = start + direction * (i * step_size)
        time = start_time + i * dt

        if i == 0:
            quat = (1.0, 0.0, 0.0, 0.0)  # Default orientation for the first point
        else:
            quat = compute_orientation(waypoints[-1]["position"], point.tolist())

        waypoints.append({
            "position": point.tolist(),
            "time": round(time, 2),
            "order": i,
            "step_size": step_size,
            "orientation": quat
        })

    end_time = start_time + num_steps * dt
    return waypoints, end_time


def generate_arc(start, end, center, direction, velocity, dt, start_time):
    """Generate waypoints along a circular arc in 3D with velocity-based spacing."""
    start = np.array(start)
    end = np.array(end)
    center = np.array(center)

    start_angle = np.arctan2(start[1] - center[1], start[0] - center[0])
    end_angle = np.arctan2(end[1] - center[1], end[0] - center[0])

    if direction == "CC" and end_angle < start_angle:
        end_angle += 2 * np.pi
    elif direction == "C" and end_angle > start_angle:
        end_angle -= 2 * np.pi

    radius = np.linalg.norm(start[:2] - center[:2])
    arc_length = np.abs(radius * (end_angle - start_angle))
    step_size = velocity * dt
    num_steps = int(arc_length / step_size)

    z_step = (end[2] - start[2]) / num_steps

    waypoints = []
    for i in range(num_steps + 1):
        angle = start_angle + i * step_size / radius * (1 if direction == "CC" else -1)
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = start[2] + i * z_step
        time = start_time + i * dt

        if i == 0:
            quat = (1.0, 0.0, 0.0, 0.0)  # Default orientation for the first point
        else:
            quat = compute_orientation(waypoints[-1]["position"], [x, y, z])

        waypoints.append({
            "position": [round(x, 6), round(y, 6), round(z, 6)],
            "time": round(time, 2),
            "order": i,
            "step_size": step_size,
            "orientation": quat
        })

    end_time = start_time + num_steps * dt
    return waypoints, end_time


def generate_path(yaml_path):
    """Generate a 3D path from a YAML file containing line and arc segments."""
    data = load_yaml(yaml_path)
    path = data["path"]
    
    all_waypoints = []
    segment_types = []

    prev_end_time = 0.0
    for segment in path:
        if segment["type"] == "line":
            waypoints, prev_end_time = generate_line(segment["start"], segment["end"], segment["velocity"], segment["dt"], prev_end_time)
        elif segment["type"] == "arc":
            waypoints, prev_end_time = generate_arc(segment["start"], segment["end"], segment["center"], segment["direction"], segment["velocity"], segment["dt"], prev_end_time)
        else:
            print(f"Unknown path segment type: {segment['type']}")
            continue
        
        all_waypoints.extend(waypoints)
        segment_types.append((segment["type"], len(waypoints)))
    
    return all_waypoints, segment_types


def save_to_csv(waypoints, segment_types, filename="auv_path.csv"):
    """Save waypoints to a CSV file with time, coordinates, orientation, and type."""
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time", "order", "x", "y", "z", "qw", "qx", "qy", "qz", "step_size", "type"])

        start_idx = 0
        for segment_type, num_points in segment_types:
            segment_label = "S" if segment_type == "line" else "T"
            
            for i in range(start_idx, start_idx + num_points):
                time = round(waypoints[i]["time"], 2)
                order = waypoints[i]["order"]
                step_size = round(waypoints[i]["step_size"], 2)
                x, y, z = map(lambda v: round(v, 4), waypoints[i]["position"])
                qw, qx, qy, qz = waypoints[i]["orientation"]
                writer.writerow([time, order, x, y, z, qw, qx, qy, qz, step_size, segment_label])

            start_idx += num_points

    print(f"Waypoints saved to {filename}")


class LeadPathGenerator(Node):
    """ROS 2 Node for generating the AUV path."""
    def __init__(self):
        super().__init__('path_generator')

        # Declare a parameter for YAML file path (default: path.yaml)
        self.declare_parameter('input_yaml', '')
        self.declare_parameter('output_csv', '')

        input_yaml = self.get_parameter('input_yaml').get_parameter_value().string_value
        output_csv = self.get_parameter('output_csv').get_parameter_value().string_value

        self.get_logger().info(f"Path Generator Node Started. Using YAML file: {input_yaml}")

        waypoints, segment_types = generate_path(input_yaml)
        save_to_csv(waypoints, segment_types, output_csv)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        plot_waypoints(waypoints, segment_types, ax)
        set_axes_equal(ax)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    LeadPathGenerator()

if __name__ == "__main__":
    main()
