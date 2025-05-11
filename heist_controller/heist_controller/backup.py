#!/usr/bin/env python3
import numpy as np
import rclpy
import time
from rclpy.node import Node
from heist_msgs.msg import HeistState
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from ackermann_msgs.msg import AckermannDriveStamped
from enum import Enum

class State(Enum):
    FOLLOW = 1
    PARK = 2
    CORRECT = 3
    END = 4

class Obj(Enum):
    BANANA_A = 1
    BANANA_B = 2
    HOME = 3

class BackupController(Node):

    def __init__(self):
        super().__init__("backup_controller")
        # Declare parameters to make them available for use
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter("drive_topic", "default")

        # Fetch constants from the ROS parameter server
        # This is necessary for the tests to be able to test varying parameters!
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.state_pub = self.create_publisher(Int32, "/change_info", 1)
        self.state_sub = self.create_subscription(HeistState, "/heist_state", self.state_callback, 1)

        self.start_time = 0
        self.started = False
        self.timer = self.create_timer(0.05, self.on_timer)

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            self.DRIVE_TOPIC,
            10)
        self.odom_sub = self.create_subscription(Odometry,
                                                 self.odom_topic,
                                                 self.pose_callback,
                                                 1)

        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.header.stamp = self.get_clock().now().to_msg()
        self.drive_msg.header.frame_id = "base_link"

        self.drive_msg.drive.steering_angle = 0.0
        self.drive_msg.drive.steering_angle_velocity = 0.0 # change as quick as possible
        self.drive_msg.drive.speed = -0.5 # m/s
        self.drive_msg.drive.acceleration = 0.0 # change as quick as possible
        self.drive_msg.drive.jerk = 0.0 # change as quick as possible

        self.known = [
                      (-5.0902533531188965, 25.827289581298828),
                      (-20.21269416809082, 25.44011116027832),
                      (-20.402408599853516, 32.01554870605469),
                      (-27.574138641357422, 33.71652603149414),
                      ]

        # Commands IF not returning home
        self.vel = [-0.5, -0.5, -0.5, 0.0]
        self.ang = [0.0, 0.0, np.deg2rad(-90), 0.0]
        self.backup_time = [2.5, 2.0, 1.0, 0.0]

        # Commands IF returning home
        self.home_vel = [0.0, -0.5, -0.5, -0.5]
        self.home_ang = [0.0, np.deg2rad(-90), np.deg2rad(90), np.deg2rad(-90)]
        self.home_backup_time = [0.0, 1.2, 1.0, 1.0]

        self.objective = None

        self.est_robot = (0.0, 0.0)
        self.est_banana = None
        self.timer_max = 0

    def state_callback(self, statemsg):
        if statemsg.state == State.CORRECT.value:
            if not self.started:
                self.started = True
                self.objective = statemsg.objective
                self.start_time = time.time()

    # def _on_delay(self):
    #     self.timer.cancel()
    #     self.get_logger().info("Backup controller activated")
    #     self.on_timer()

    def get_closest(self, px):
        self.get_logger().info(f"robot pose: {px}")
        self.get_logger().info(f"known: {self.known}")
        smallest_dist = 10000000000000000000
        best_i = 0
        for i, pt in enumerate(self.known):
            dist = ((pt[0]-px[0])**2 + (pt[1]-px[1])**2) ** (1/2)
            if dist < smallest_dist:
                smallest_dist = dist
                best_i = i
        return best_i

    def pose_callback(self, odometry_msg):
        robot_position = odometry_msg.pose.pose.position
        robot_x = robot_position.x
        robot_y = robot_position.y

        self.est_robot = (robot_x, robot_y)

    def on_timer(self):
        # self.get_logger().info(f"{self.started}")
        if self.started:
            if self.est_banana is None:
                self.est_banana = self.get_closest(self.est_robot)

                if self.objective == Obj.BANANA_A.value:
                    self.get_logger().info("entered next banana backup")
                    self.drive_msg.drive.speed = self.vel[self.est_banana]
                    self.drive_msg.drive.steering_angle = self.ang[self.est_banana]
                    self.timer_max = self.backup_time[self.est_banana]
                elif self.objective == Obj.BANANA_B.value:
                    self.get_logger().info("entered going home backup")
                    self.get_logger().info(f"at {self.est_banana}")
                    self.drive_msg.drive.speed = self.home_vel[self.est_banana]
                    self.drive_msg.drive.steering_angle = self.home_ang[self.est_banana]
                    self.timer_max = self.home_backup_time[self.est_banana]

            self.drive_pub.publish(self.drive_msg)

            if time.time() - self.start_time > self.timer_max:
                msg = Int32()
                msg.data = State.CORRECT.value
                self.state_pub.publish(msg)
                self.started = False
                self.est_banana = None

def main():
    rclpy.init()
    backup_controller = BackupController()
    rclpy.spin(backup_controller)
    backup_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
