import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, PointStamped, Pose, PoseArray
from std_msgs.msg import Int32
from heist_msgs.msg import HeistState
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

class HeistStatePublisher(Node):
    '''
    '''
    def __init__(self):
        super().__init__("HeistStatePub")
        self.publisher = self.create_publisher(HeistState, "/heist_state", 1)
        self.subscriber = self.create_subscription(Int32, "/change_info", self.callback, 1)

        self.state = State.FOLLOW.value
        self.obj = Obj.BANANA_A.value

        self.state_msg = HeistState()
        self.state_msg.state = self.state
        self.state_msg.objective = self.obj
        self.get_logger().info(f"Entered If, {self.state}, {self.obj}")
        self.publisher.publish(self.state_msg)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("State Machine Publisher Initialized")

    def timer_callback(self):
        self.publisher.publish(self.state_msg)

    def callback(self, msg: Int32):
        if msg.data == self.state:
            self.go_next_state()

            self.state_msg.state = self.state
            self.state_msg.objective = self.obj
            self.get_logger().info(f"Entered If, state: {self.state}, objective: {self.obj}")
            self.publisher.publish(self.state_msg)
        self.get_logger().info("CALLBACK")

    def go_next_state(self):
        if self.state == State.FOLLOW.value and self.obj == Obj.HOME.value:
            self.state = State.END.value
            return

        self.state += 1
        if self.state > State.CORRECT.value:
            self.state = State.FOLLOW.value
            self.obj += 1

def main(args=None):
    rclpy.init(args=args)
    node = HeistStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
