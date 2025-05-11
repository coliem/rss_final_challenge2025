import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from .detector import Detector

from .red_segmentation import cd_color_segmentation
from ackermann_msgs.msg import AckermannDriveStamped

class RedStop(Node):
    def __init__(self):
        super().__init__('red_stop')

        self.declare_parameter("drive_topic", "/vesc/low_level/input/safety") #"/vesc/low_level/input/safety"
        DRIVE_TOPIC = self.get_parameter("drive_topic").value
        self.detector = Detector()


        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 10)
        self.red_image_debug_pub = self.create_publisher(Image, "/red_light/image_debug", 10)
        self.cropped_image_debug_pub = self.create_publisher(Image, "/red_light/crop_debug", 10)

        self.stop_msg = AckermannDriveStamped()
        self.stop_msg.header.stamp = self.get_clock().now().to_msg()
        self.stop_msg.header.frame_id = "base_link"
        self.stop_msg.drive.steering_angle = 0.0
        self.stop_msg.drive.steering_angle_velocity = 0.0
        self.stop_msg.drive.speed = 0.0
        self.stop_msg.drive.acceleration = 0.0
        self.stop_msg.drive.jerk = 0.0

        # for testing
        # self.timer = self.create_timer(0.05, self.on_timer)
        # self.test_drive_pub = self.create_publisher(AckermannDriveStamped, "/vesc/high_level/input/nav_0", 10)

        self.bridge = CvBridge()

    def image_callback(self, image_msg: Image):
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image_copy = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        model_out = self.detector.predict(image)
        debug_img = model_out
        predictions =  model_out["predictions"]
        t_light = [p for p in predictions if p[-1] == "traffic light"]
        if t_light:
            x1, y1, x2, y2 = t_light[0][0]
            cropped_image = image_copy[int(y1)-10:int(y2)+10, int(x1)-10:int(x2)+10]
            if cropped_image is not None and cropped_image.size != 0:
                bounding_box, red_present = cd_color_segmentation(cropped_image, None)
                if red_present:
                    self.get_logger().info("Red light detected")

                    # stop the car
                    self.drive_pub.publish(self.stop_msg)

                    cv2.rectangle(image_copy, bounding_box[0], bounding_box[1], (255,0,255), 2)
                    cv2.putText(image_copy, "RED LIGHT", (240, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)
                debug_msg = self.bridge.cv2_to_imgmsg(np.array(cropped_image), "bgr8")
                self.cropped_image_debug_pub.publish(debug_msg)


        else:
            # self.get_logger().info("No red light")
            # self.get_logger().info("No red light detected")
            cv2.putText(image_copy, "NO RED LIGHT", (220, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,0,255), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(image_copy, "bgr8")
        self.red_image_debug_pub.publish(debug_msg)

    # def on_timer(self):
    #     self.test_drive_msg = AckermannDriveStamped()
    #     self.test_drive_msg.header.stamp = self.get_clock().now().to_msg()
    #     self.test_drive_msg.header.frame_id = "base_link"
    #     self.test_drive_msg.drive.steering_angle = 0.0
    #     self.test_drive_msg.drive.steering_angle_velocity = 0.0
    #     self.test_drive_msg.drive.speed = 1.0
    #     self.test_drive_msg.drive.acceleration = 0.0
    #     self.test_drive_msg.drive.jerk = 0.0
    #     self.test_drive_pub.publish(self.test_drive_msg)

    #     self.get_logger().info("Driving")

def main(args=None):
    rclpy.init(args=args)
    red_stop = RedStop()
    rclpy.spin(red_stop)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
