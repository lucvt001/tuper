import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
import tf2_ros

class CSVTFBroadcaster(Node):
    """ROS 2 Node to read position & orientation from a CSV file and broadcast as a TF transform."""

    def __init__(self):
        super().__init__('csv_tf_broadcaster')

        # Declare parameters
        self.declare_parameter('csv_file', '')
        self.declare_parameter('child_frame', '')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('rate', 10.0)  # Hz
        self.declare_parameter('path_topic', '')

        # Get parameter values
        self.csv_file = self.get_parameter('csv_file').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value

        self.get_logger().info(f"Broadcasting TF from {self.csv_file} [{self.parent_frame} → {self.child_frame}] at {self.rate} Hz")
        self.get_logger().info(f"Publishing Path to topic: {self.path_topic}")

        # Load CSV data
        self.poses = self.load_csv(self.csv_file)
        if not self.poses:
            self.get_logger().error("No valid poses loaded. Shutting down node.")
            rclpy.shutdown()
            return

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Path Publisher
        self.path_publisher = self.create_publisher(Path, self.path_topic, 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.parent_frame  # Path is in the parent frame

        # Timer for publishing TF at a fixed rate
        self.index = 0  # Keeps track of the current row in CSV
        self.timer = self.create_timer(1.0 / self.rate, self.broadcast_tf)

    def load_csv(self, filename):
        """Loads position & orientation from a CSV file."""
        poses = []
        try:
            with open(filename, mode='r') as file:
                reader = csv.DictReader(file)
                for row in reader:
                    try:
                        time_stamp = float(row["time"])
                        x, y, z = float(row["x"]), float(row["y"]), float(row["z"])
                        qw, qx, qy, qz = float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])
                        poses.append({
                            "time": time_stamp,
                            "position": (x, y, z),
                            "orientation": (qw, qx, qy, qz)
                        })
                    except ValueError:
                        self.get_logger().warn(f"Skipping invalid row: {row}")
        except FileNotFoundError:
            self.get_logger().error(f"CSV file not found: {filename}")
            return []
        
        self.get_logger().info(f"Loaded {len(poses)} poses from {filename}")
        return poses

    def broadcast_tf(self):
        """Broadcasts TF transforms from the CSV data."""
        if self.index >= len(self.poses):
            self.get_logger().info("Reached end of CSV file.")
            rclpy.shutdown()
            return

        pose = self.poses[self.index]
        self.index += 1  # Move to next pose

        # Broadcast TF
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = pose["position"][0]
        transform.transform.translation.y = pose["position"][1]
        transform.transform.translation.z = pose["position"][2]
        transform.transform.rotation.w = pose["orientation"][0]
        transform.transform.rotation.x = pose["orientation"][1]
        transform.transform.rotation.y = pose["orientation"][2]
        transform.transform.rotation.z = pose["orientation"][3]

        self.tf_broadcaster.sendTransform(transform)

        # Append to Path and Publish
        pose_stamped = PoseStamped()
        pose_stamped.header = transform.header  # Use the same timestamp & frame
        pose_stamped.pose.position.x = pose["position"][0]
        pose_stamped.pose.position.y = pose["position"][1]
        pose_stamped.pose.position.z = pose["position"][2]
        pose_stamped.pose.orientation.w = pose["orientation"][0]
        pose_stamped.pose.orientation.x = pose["orientation"][1]
        pose_stamped.pose.orientation.y = pose["orientation"][2]
        pose_stamped.pose.orientation.z = pose["orientation"][3]

        self.path_msg.header.stamp = transform.header.stamp  # Update path header timestamp
        self.path_msg.poses.append(pose_stamped)  # Append new pose to the path
        self.path_publisher.publish(self.path_msg)  # Publish the updated path


def main(args=None):
    rclpy.init(args=args)
    node = CSVTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
