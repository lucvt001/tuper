import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, Twist
from sam_msgs.msg import ThrusterAngles
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from filterpy.common import Q_discrete_white_noise
from functools import partial
from collections import deque

class PositionFilter(Node):

    def __init__(self):
        super().__init__('position_filter')

        # Initialize class variables
        self.prev_heading, self.current_heading = None, None
        self.rpm = 0
        self.horizontal_thrust_vector = 0.
        self.dt = 0.1

        # Offsets
        self.leader1_offset = np.array([0., 0.])
        self.leader2_offset = np.array([0., 10.])

        # Subscribers containing the range of the follower from the leaders
        self.leader1_distance_sub = self.create_subscription(Float32, "/follower/distance_to_leader1", self.leader1_distance_cb, 1)
        self.leader2_distance_sub = self.create_subscription(Float32, "/follower/distance_to_leader2", self.leader2_distance_cb, 1)

        # Publisher to publish the filtered data of the follower
        self.filtered_position_pub = self.create_publisher(Point, '/follower/filtered_position', 1)
        self.filtered_velocity_pub = self.create_publisher(Twist, '/follower/filtered_velocity', 1)

        # Timer for predict step of the filter
        self.timer = self.create_timer(self.dt, self.filter_predict)

        # Initialize the filter
        # State: [pos_x, vel_x, pos_y, vel_y]
        # Measurement: [distance_to_leader1], or [distance_to_leader2]  

        self.filter_reset()
        self.prev_update_time = self.get_clock().now()

        self.rolling_pos_x = deque(maxlen=4)
        self.rolling_pos_y = deque(maxlen=4)
        self.wma_weights = np.array([0.1, 0.2, 0.2, 0.5])

    def filter_reset(self):
        points = MerweScaledSigmaPoints(n=4, alpha=1e-2, beta=2., kappa=-1.0)
        self.ukf = UnscentedKalmanFilter(dim_x=4, dim_z=1, dt=self.dt, points=points, fx=self.state_transition_function, hx=None)    
        self.ukf.x = np.array([-11., 0.0, 3.0, 0.0])
        self.ukf.P = np.diag([9., 4., 9., 4.])
        self.ukf.Q = Q_discrete_white_noise(dim=2, dt=1, var=0.6**2, block_size=2)  # Process noise. Choose dt=1 because dt=0.1 cause the Q matrix to be too small, leading to numerical issues
        self.ukf.R = np.array([0.3**2]) # Measurement noise

    def filter_predict(self):
        self.ukf.predict()
        # self.get_logger().info(f'UKF state: {self.ukf.x.tolist()}')

    def state_transition_function(self, x: np.ndarray, dt: float):
        pos_x, vel_x, pos_y, vel_y = x
        pos_x += vel_x * dt
        pos_y += vel_y * dt
        return np.array([pos_x, vel_x, pos_y, vel_y])

    def leader1_distance_cb(self, msg: Float32):
        time_now = self.get_clock().now()
        time_elapsed = (time_now - self.prev_update_time).nanoseconds / 1e9
        if time_elapsed <= 0.11:    # to avoid updating the filter many times at once which can cause it to collapse
            return 
        distance_to_leader1 = msg.data
        self.update_and_publish(np.array([distance_to_leader1]), self.leader1_offset)
        self.prev_update_time = time_now

    def leader2_distance_cb(self, msg: Float32):
        time_now = self.get_clock().now()
        time_elapsed = (time_now - self.prev_update_time).nanoseconds / 1e9
        if time_elapsed <= 0.11:
            return       
        distance_to_leader2 = msg.data
        self.update_and_publish(np.array([distance_to_leader2]), self.leader2_offset)     
        self.prev_update_time = time_now

    def measurement_function(self, x: np.ndarray, offset: np.ndarray):
        pos_x, _, pos_y, _ = x
        return np.sqrt([ (pos_x-offset[0]) ** 2 + (pos_y-offset[1]) ** 2 ])
    
    def update_and_publish(self, measurement: np.ndarray, offset: np.ndarray):
        self.ukf.hx = partial(self.measurement_function, offset=offset)
        self.ukf.update(z=measurement)
        # if abs(self.ukf.x[0]) < 0.2:
        #     self.get_logger().warn(f'UKF diverged: {self.ukf.x.tolist()}. Resetting filter.')
        #     self.filter_reset()
        self.publish_data()
        # self.get_logger().info(f'UKF updated state: {self.ukf.x.tolist()}')

    def publish_data(self):
        pos_x, vel_x, pos_y, vel_y = self.ukf.x
        pos_x = self.wma(pos_x, self.rolling_pos_x, self.wma_weights)
        pos_y = self.wma(pos_y, self.rolling_pos_y, self.wma_weights)
        point = Point()
        point.x = pos_x
        point.y = pos_y
        point.z = 0.0  # Assuming z is 0 for this example
        self.filtered_position_pub.publish(point)
        # self.get_logger().info(f'Publishing Point: x={point.x}, y={point.y}, z={point.z}')

        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y
        self.filtered_velocity_pub.publish(twist)

    def wma(self, data: float, prev_data: deque, weights: np.ndarray) -> float:
        prev_data.append(data)
        if len(prev_data) < 4:
            return data
        data_array = np.array(prev_data)
        smoothened_data = np.dot(weights, data_array)
        prev_data.append(data)
        return smoothened_data

def main(args=None):
    rclpy.init(args=args)
    position_filter = PositionFilter()
    rclpy.spin(position_filter)
    position_filter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()