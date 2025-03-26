import csv
import yaml
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from .utils import plot_waypoints, set_axes_equal
from tf_transformations import quaternion_from_euler

def load_leader_path(csv_filename: str) -> tuple[list[dict], list[tuple[str, int]]]:
    """
    Reads the leader's path from a CSV file.
    Expected CSV columns: time, x, y, z, type, order, step_size.
    Returns a list of waypoints (as dicts) and the leader's segment types.
    """
    waypoints = []
    segment_types = []
    with open(csv_filename, mode="r") as file:
        reader = csv.DictReader(file)
        current_seg_type = None
        seg_count = 0
        for row in reader:
            wp = {
                "time": float(row["time"]),
                "position": (float(row["x"]), float(row["y"]), float(row["z"])),
                "type": row["type"],
                "order": int(row["order"]),
                "step_size": float(row["step_size"])
            }
            waypoints.append(wp)
            if current_seg_type is None:
                current_seg_type = wp["type"]
                seg_count = 1
            else:
                if wp["order"] == 0:
                    segment_types.append((current_seg_type, seg_count))
                    current_seg_type = wp["type"]
                    seg_count = 1
                else:
                    seg_count += 1
        if current_seg_type is not None:
            segment_types.append((current_seg_type, seg_count))
    return waypoints, segment_types

def load_offset(yaml_filename: str) -> tuple[float, float, float]:
    """
    Reads the follower offset from a YAML file.
    YAML structure example:
      offset: [x_offset, y_offset, 0.0]
    Returns a tuple (x_offset, y_offset, z_offset)
    """
    with open(yaml_filename, "r") as file:
        data = yaml.safe_load(file)
    return tuple(data["offset"])

def compute_cumulative_distance(leader_waypoints: list[dict]) -> list[float]:
    """
    Compute the cumulative distance along the leader's path.
    Assumes that within a segment, the distance between consecutive points is step_size.
    When a segment starts (order==0 and not the first waypoint), the cumulative distance remains continuous.
    Returns a list of cumulative distances for each waypoint.
    """
    cum_dist = []
    total = 0.0
    for i, wp in enumerate(leader_waypoints):
        if i == 0:
            total = 0.0
            cum_dist.append(total)
        else:
            if wp["order"] == 0:
                cum_dist.append(total)
            else:
                total += wp["step_size"]
                cum_dist.append(total)
    return cum_dist

def find_interpolated_point(cum_dist: list[float], leader_waypoints: list[dict], target_distance: float) -> tuple[tuple[float, float, float], np.ndarray]:
    """
    Given the cumulative distances and leader waypoints, find the position and tangent
    corresponding to the given target_distance (which may lie between two leader points).
    If target_distance is below 0, returns the first waypoint.
    Returns (ref_point, tangent) where:
      - ref_point is a 3-tuple (x,y,z)
      - tangent is the unit vector (in XY plane) of the local path direction.
    """

    # Clamp target_distance to not be negative
    if target_distance <= 0:
        pos = leader_waypoints[0]["position"]
        next_pos = leader_waypoints[1]["position"]
        diff = np.array(next_pos) - np.array(pos)
        tangent = diff[:2]
        norm = np.linalg.norm(tangent)
        tangent = tangent / norm if norm > 0 else np.array([1, 0])
        return pos, tangent

    # Find indices j such that cum_dist[j] <= target_distance <= cum_dist[j+1]
    for j in range(len(cum_dist) - 1):
        if cum_dist[j] <= target_distance <= cum_dist[j + 1]:
            frac = (target_distance - cum_dist[j]) / (cum_dist[j + 1] - cum_dist[j])
            p0 = np.array(leader_waypoints[j]["position"])
            p1 = np.array(leader_waypoints[j + 1]["position"])
            ref_point = p0 + frac * (p1 - p0)
            diff = p1 - p0
            tangent = diff[:2]
            norm = np.linalg.norm(tangent)
            tangent = tangent / norm if norm > 0 else np.array([1, 0])
            return tuple(ref_point), tangent
        
    # If target_distance exceeds the last cumulative distance, return the last point.
    pos = leader_waypoints[-1]["position"]
    # Use the previous segment for tangent
    p0 = np.array(leader_waypoints[-2]["position"])
    p1 = np.array(pos)
    diff = p1 - p0
    tangent = diff[:2]
    norm = np.linalg.norm(tangent)
    tangent = tangent / norm if norm > 0 else np.array([1, 0])
    return pos, tangent

def generate_follower_path(leader_waypoints: list[dict], offset: tuple[float, float, float]) -> tuple[list[dict], list[tuple[str, int]]]:
    """
    Generates the follower's path using the leader's waypoints and a given offset (x, y, z).
    The follower's desired point at leader waypoint i is defined as the leader's point at a cumulative
    distance (d + offset_x), where d is the cumulative distance at i.
    Lateral offset (offset_y) is applied perpendicular to the local path tangent.
    Returns:
      follower_waypoints: list of dicts with keys: time, position, type
      follower_segment_types: list of tuples: (type, count)
    """
    x_offset, y_offset, z_offset = offset
    follower_waypoints = []
    cum_dist = compute_cumulative_distance(leader_waypoints)

    for i, wp in enumerate(leader_waypoints):
        d_leader = cum_dist[i]
        # Compute the target cumulative distance for the follower
        target_d = d_leader + x_offset
        ref_point, tangent = find_interpolated_point(cum_dist, leader_waypoints, target_d)

        # Compute perpendicular direction in XY plane (rotate tangent by +90 degrees)
        # Here, if tangent = (tx, ty), then perp = (-ty, tx)
        perp = np.array([-tangent[1], tangent[0]])
        # Follower position = reference point + (offset_y * perpendicular)
        ref_point = np.array(ref_point)
        follower_xy = ref_point[:2] + y_offset * perp
        follower_pos = (follower_xy[0], follower_xy[1], ref_point[2] + z_offset)

        yaw = np.arctan2(tangent[1], tangent[0])
        quaternion = quaternion_from_euler(0, 0, yaw)

        follower_waypoints.append({
            "time": wp["time"],
            "position": follower_pos,
            "type": wp["type"],
            "quaternion": quaternion
        })

    # Now, compute segment types from the follower waypoints (group consecutive ones)
    follower_segment_types = []
    if follower_waypoints:
        current_type = follower_waypoints[0]["type"]
        count = 0
        for wp in follower_waypoints:
            if wp["type"] == current_type:
                count += 1
            else:
                follower_segment_types.append((current_type, count))
                current_type = wp["type"]
                count = 1
        follower_segment_types.append((current_type, count))

    return follower_waypoints, follower_segment_types

def save_follower_path(follower_waypoints: list[dict], filename: str):
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time", "x", "y", "z", "qx", "qy", "qz", "qw", "type"])
        for wp in follower_waypoints:
            time = round(wp["time"], 2)
            x, y, z = map(lambda v: round(v, 2), wp["position"])
            qx, qy, qz, qw = map(lambda q: round(q, 6), wp["quaternion"])
            writer.writerow([time, x, y, z, qx, qy, qz, qw, wp["type"]])
    print(f"Follower path saved to {filename}")

class OffsetPathGenerator(Node):
    def __init__(self):
        super().__init__("path_generator")
        node_name = self.get_name()

        self.declare_parameter("leader_csv", "leader_path.csv")
        self.declare_parameter("offset_yaml", "offset.yaml")
        self.declare_parameter("output_csv", "follower_path.csv")
        self.declare_parameter("do_plot", False)

        leader_csv = self.get_parameter("leader_csv").get_parameter_value().string_value
        offset_yaml = self.get_parameter("offset_yaml").get_parameter_value().string_value
        output_csv = self.get_parameter("output_csv").get_parameter_value().string_value
        do_plot = self.get_parameter("do_plot").get_parameter_value().bool_value

        leader_waypoints, leader_segment_types = load_leader_path(leader_csv)
        offset = load_offset(offset_yaml)
        self.get_logger().info(f"Loaded offset: {offset}")

        follower_waypoints, follower_segment_types = generate_follower_path(leader_waypoints, offset)
        save_follower_path(follower_waypoints, output_csv)

        if do_plot:
            fig = plt.figure(node_name)
            ax = fig.add_subplot(111, projection='3d')
            plot_waypoints(follower_waypoints, follower_segment_types, ax)
            set_axes_equal(ax)
            plt.show()

def main(args=None):
    rclpy.init(args=args)
    OffsetPathGenerator()

if __name__ == "__main__":
    main()
