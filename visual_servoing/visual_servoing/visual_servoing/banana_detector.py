import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from heist_msgs.msg import HeistState
from vs_msgs.msg import ConeLocationPixel
import os
from .detector import Detector
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from enum import Enum

##NEED TEST AGAIN

class State(Enum):
    FOLLOW = 1
    PARK = 2
    CORRECT = 3
    END = 4

class DetectorNode(Node):
    def __init__(self):
        super().__init__("banana_detector")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
        self.declare_parameter('full_run', True)
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar
        self.full_run = self.get_parameter('full_run').get_parameter_value().bool_value

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.detector = Detector()
        self.debug_pub = self.create_publisher(Image, "/banana_debug_img", 10)
        self.publisher = self.create_publisher(ConeLocationPixel, "/relative_banana_px", 10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.state_pub = self.create_publisher(Int32, "/change_info", 1)
        self.state_sub = self.create_subscription(HeistState, "/heist_state", self.state_callback, 1)
        self.valid_state = False
        self.prev_goal = None
        self.bridge = CvBridge()

        # self.last_banana = None
        self.saved_img = False

        self.get_logger().info("Detector Initialized")

    def state_callback(self, statemsg):
        """
        Used to determine when we should run inference with camera. If the
        state is a valid state (when we could potentiatlly see banana)
        then we should try to detect banana
        """
        if statemsg.state == State.PARK.value:
            self.valid_state = True
        else:
            self.valid_state = False

        if statemsg.objective != self.prev_goal:
            self.saved_img = False
            self.prev_goal = statemsg.objective

    def callback(self, img_msg):
        if (self.full_run and self.valid_state is True) or (not self.full_run):
            # Process image with CV Bridge
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            model_out = self.detector.predict(image)
            debug_img = model_out
            predictions =  model_out["predictions"]
            # self.get_logger().info(predictions)
            bananas = [p for p in predictions if p[-1] in ["banana"]]
            if bananas:
                cone_px = ConeLocationPixel()
                drive_cmd = AckermannDriveStamped()

                banana_coords = bananas[0][0]
                bottomv = float(banana_coords[3])
                bottomu = float(banana_coords[0] + banana_coords[2])//2
                cone_px.u, cone_px.v = bottomu, bottomv
                self.last_banana = banana_coords
                self.publisher.publish(cone_px)


                # drive_cmd.header.stamp = self.get_clock().now().to_msg()
                # drive_cmd.header.frame_id = "base_link"

                # drive_cmd.drive.steering_angle = 0.
                # drive_cmd.drive.steering_angle_velocity = 0.0
                # drive_cmd.drive.speed = 0.
                # drive_cmd.drive.acceleration = 0.
                # drive_cmd.drive.jerk = 0.
                # self.drive_pub.publish(drive_cmd)
                original_image = model_out["original_image"]
                out = self.detector.draw_box(original_image, predictions, draw_all=True)
                debug_msg = self.bridge.cv2_to_imgmsg(np.array(out), "bgr8")
                self.debug_pub.publish(debug_msg)


                if not self.saved_img:
                    save_path = f"{os.path.dirname(__file__)}/tf_output_{self.prev_goal}.png"
                    out.save(save_path)
                    self.saved_img = True
            else:
                original_image = model_out["original_image"]
                out = self.detector.draw_box(original_image, predictions, draw_all=True)
                debug_msg = self.bridge.cv2_to_imgmsg(np.array(out), "bgr8")
                self.debug_pub.publish(debug_msg)


            # elif self.last_banana is not None:
            #     cone_px = ConeLocationPixel()
            #     bottomv = float(self.last_banana[3])r
            #     bottomu = float(self.last_banana[0] + self.last_banana[2])//2
            #     cone_px.u, cone_px.v = bottomu, bottomv
            #     self.publisher.publish(cone_px)



def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
