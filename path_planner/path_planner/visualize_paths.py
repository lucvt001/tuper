import csv
import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from .utils import plot_waypoints, set_axes_equal  # Assuming utils.py is in the package


def load_path(csv_filename):
    """
    Reads a CSV file containing the path.
    Expected CSV columns: time, x, y, z, type.
    Returns:
      - waypoints: List of dicts containing time, position, and type.
      - segment_types: List of (segment_type, num_points) tuples.
    """
    waypoints = []
    segment_types = []
    prev_type = None
    count = 0

    try:
        with open(csv_filename, mode="r") as file:
            reader = csv.DictReader(file)
            for row in reader:
                point = {
                    "time": float(row["time"]),
                    "position": (round(float(row["x"]), 2), round(float(row["y"]), 2), round(float(row["z"]), 2)),
                    "type": row["type"]  # 'S' for straight, 'T' for turn
                }
                waypoints.append(point)

                # Track segment types
                if prev_type is None:
                    prev_type = point["type"]
                    count = 1
                elif prev_type == point["type"]:
                    count += 1
                else:
                    segment_types.append((prev_type, count))
                    prev_type = point["type"]
                    count = 1

            # Append the last segment
            if count > 0:
                segment_types.append((prev_type, count))

    except FileNotFoundError:
        print(f"Error: CSV file not found: {csv_filename}")
        return [], []

    return waypoints, segment_types


class PathVisualizerNode(Node):
    """ROS 2 Node for visualizing multiple AUV paths from CSV files."""

    def __init__(self):
        super().__init__('path_visualizer')

        # Declare parameter to accept a list of CSV files
        self.declare_parameter('csv_files', ["", ""])

        # Get the list of CSV files
        csv_files = self.get_parameter('csv_files').get_parameter_value().string_array_value

        if not csv_files:
            self.get_logger().warn("No CSV files provided! Set the 'csv_files' parameter.")
            return

        self.get_logger().info(f"Loading {len(csv_files)} paths for visualization.")

        # Create 3D plot
        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection="3d")

        # Load and plot each CSV file
        for csv_file in csv_files:
            waypoints, segment_types = load_path(csv_file)
            if waypoints:
                self.get_logger().info(f"Plotting: {csv_file}")
                plot_waypoints(waypoints, segment_types, ax)
            else:
                self.get_logger().warn(f"Skipping empty or invalid file: {csv_file}")

        set_axes_equal(ax)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    PathVisualizerNode()

if __name__ == "__main__":
    main()
