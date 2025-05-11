#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety")
        # Declare parameters to make them available for use
        self.get_logger().info("entered safety init")
        self.declare_parameter("cmd_topic", "default")
        self.declare_parameter("laser_topic", "default")
        self.declare_parameter("drive_topic", "default")

        # Fetch constants from the ROS parameter server
        self.CMD_TOPIC = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.LASER_TOPIC = self.get_parameter('laser_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.cmd_sub = self.create_subscription(
            AckermannDriveStamped,
            self.CMD_TOPIC,
            self.listener_callback,
            10)
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.LASER_TOPIC,
            self.laser_callback,
            10)
        self.safety_pub = self.create_publisher(
            AckermannDriveStamped,
            self.DRIVE_TOPIC,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            "/pf/pose/odom",
            self.pose_callback,
            10)

        self.prev_cmd = None
        self.angle_memory = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.weights = [0.1, 0.2, 0.3, 0.4, 0.5]

        # Data to be saved by laser_callback()
        self.angles = np.array([])
        self.ranges = np.array([])

        # PARAMETERS TODO CAN CHANGE
        # Delta time, assumed increment for predicting where robot will be
        # 0.5 -> 0.25
        self.DT = 1 # in seconds
        # Angle range, range of laserscan data we will scan over
        # rangle calculated as drive_command.steering_angle +/- ang_range
        # 0.2 -> 0.15 -> 0.1 -> 0.075 -> 0.15 -> .2 -> 0.1
        self.ang_range = 0.1 # in radians
        # Tolerance, car will stop if predicted distance from wall is <= tolerance
        self.dist_tolerance = 0.75 # in meters
        # Danger threshold, car will stop if this % of range data reads
        # within the dist_ range
        self.danger_threshold = 0.1 # 20 percent

        self.stop_msg = AckermannDriveStamped()
        self.stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.stop_msg.header.frame_id = "base_link"

        self.stop_msg.drive.steering_angle = 0.0 # set everything else to 0
        self.stop_msg.drive.steering_angle_velocity = 0.0 # set everything else to 0
        self.stop_msg.drive.speed = 0.0 # STOP THE CAR
        self.stop_msg.drive.acceleration = 0.0 # set everything else to 0
        self.stop_msg.drive.jerk = 0.0 # set everything else to 0

        self.crossing = False
        self.get_logger().info("initialized safety controller")
    def pose_callback(self, msg):
        """
        Odometry callback to check if we're in TA crossing
        """
        # (-5.759727478027344, 15.125312805175781), px (629, 661)
        # (-10.710579872131348, 21.428972244262695), px (727, 535)

        if -10.71 <= msg.pose.pose.position.x <= -5.76 and 15.13 <= msg.pose.pose.position.y <= 21.43:
            # self.get_logger().info(f"IN CROSSING")
            self.crossing = True
        else:
            # self.get_logger().info(f"OUT OF CROSSING")
            self.crossing = False
    def laser_callback(self, msg):
        # Save most recent laser data
        self.angles = np.linspace(start=msg.angle_min,
            stop=msg.angle_max,
            num=int(np.round((msg.angle_max-msg.angle_min)/msg.angle_increment+1)),
            endpoint=True)

        self.ranges = np.array(msg.ranges)
        self.ranges = np.clip(self.ranges, a_min=msg.range_min, a_max=msg.range_max)
        self.printed = False

    def listener_callback(self, msg):
        # self.get_logger().info(f"Entered callback")
        if self.crossing:
            if self.prev_cmd is not None:
                drive_ang = self.prev_cmd.steering_angle
                drive_speed = self.prev_cmd.speed

                self.angle_memory[:-1] = self.angle_memory[1:]
                self.angle_memory[-1] = drive_ang
                drive_ang_adj = np.sum(self.angle_memory*self.weights)

                predicted_dist = drive_speed * self.DT
                min_safe_dist = predicted_dist + self.dist_tolerance

                inds_to_check = np.where((self.angles >= drive_ang_adj - self.ang_range) &
                                        (self.angles <= drive_ang_adj + self.ang_range))
                ranges_to_check = self.ranges[inds_to_check]
                danger_rating = np.sum(ranges_to_check < min_safe_dist) / float(ranges_to_check.size)

                # self.get_logger().info(f"{self.danger_threshold}, {danger_rating}")
                if danger_rating > self.danger_threshold:
                    if not self.printed:
                        self.get_logger().info(f"STOPPED!")
                        self.printed = True

                    self.safety_pub.publish(self.stop_msg)
                else:
                    self.printed = False
            self.prev_cmd = msg.drive

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
