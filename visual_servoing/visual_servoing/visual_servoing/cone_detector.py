#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from vs_msgs.msg import ConeLocationPixel

from std_msgs.msg import Bool

# import your color segmentation algorithm; call this function in ros_image_callback!
from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = True

        # Subscribe to ZED camera RGB frames
        self.cone_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.edge_case_pub = self.create_publisher(Bool, "/edge_case_detector", 10)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Cone Detector Initialized")

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################
        conepx = ConeLocationPixel()

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        # 640 x 360
        if self.LineFollower:
            # cv2.rectangle(image, (0,0), (640, 150), [0, 0, 0], -1)
            # cv2.rectangle(image, (270, 0), (370, 360), [0,0,0], -1)
            cv2.rectangle(image, (0,0), (640, 160), [0, 0, 0], -1) # TOP
            cv2.rectangle(image, (0, 0), (60, 360), [0,0,0], -1) # left
            cv2.rectangle(image, (580, 0), (640, 360), [0,0,0], -1) # right
            cv2.rectangle(image, (0,270), (640, 360), [0,0,0], -1) # bot
            cv2.rectangle(image, (260, 0), (380, 360), [0,0,0], -1) #mid
            #cv2.rectangle(image, (0, 275), (640, 360), [255,255,255], -1)

        bbox = cd_color_segmentation(image, None)
        # bottomv = float(bbox[1][1])
        # bottomu = float(bbox[1][0] + bbox[0][0])//2

        # Assumption input:
        # bbox for two white lines.

        bbox1 = bbox[0]
        bbox2 = bbox[1]

        box1_pt1 = bbox1[0]
        box1_pt2 = bbox1[1]
        box2_pt1 = bbox2[0]
        box2_pt2 = bbox2[1]

        # Find the first line
        # self.get_logger().info(f'p11: {float(box1_pt1[0]) }, p12: {float(box1_pt1[1])}, p13: {float(box1_pt2[0])}, p14: {float(box1_pt2[1])}')
        # self.get_logger().info(f'p21: {float(box2_pt1[0]) }, p22: {float(box2_pt1[1])}, p23: {float(box2_pt2[0])}, p24: {float(box2_pt2[1])}')
        try:
            line1_slope = (float(box1_pt2[1]) - float(box1_pt1[1])) / (float(box1_pt2[0]) - float(box1_pt1[0]))
            line1_intercept = float(box1_pt1[1]) - line1_slope * float(box1_pt1[0])
        except:
            # x_intersect = 440 #420 -> 440
            # y_intersect = image.shape[0]/2
            # conepx.u, conepx.v = float(x_intersect), float(y_intersect)

            # self.cone_pub.publish(conepx)
            # bbox_top_left = (int(x_intersect - 10) , int(y_intersect - 10))
            # bbox_bot_right = (int(x_intersect + 10), int(y_intersect + 10))
            # cv2.rectangle(image, bbox_top_left, bbox_bot_right, (0,255,0), 2)
            # # cv2.rectangle(image, box1_pt1, box1_pt2, (0,0,255), 2)
            # # cv2.rectangle(image, box2_pt1, box2_pt2, (0,0,255), 2)
            # debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            # self.debug_pub.publish(debug_msg)
            debug_msg = Bool(data = True)
            self.edge_case_pub.publish(debug_msg)

            cv2.putText(image, "No Box", (220, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            debug_msg_pict = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg_pict)

            return
        try:
            # Find the second line
            line2_slope = (float(box2_pt2[1]) - float(box2_pt1[1])) / (float(box2_pt2[0]) - float(box2_pt1[0]))
            line2_intercept = float(box2_pt1[1]) - line2_slope * float(box2_pt1[0])
        except:
            # x_intersect = 200 #420 -> 440
            # y_intersect = image.shape[0]/2
            # conepx.u, conepx.v = float(x_intersect), float(y_intersect)

            # self.cone_pub.publish(conepx)
            # bbox_top_left = (int(x_intersect - 10) , int(y_intersect - 10))
            # bbox_bot_right = (int(x_intersect + 10), int(y_intersect + 10))
            # cv2.rectangle(image, bbox_top_left, bbox_bot_right, (0,255,0), 2)
            # # cv2.rectangle(image, box1_pt1, box1_pt2, (0,0,255), 2)
            # # cv2.rectangle(image, box2_pt1, box2_pt2, (0,0,255), 2)
            # debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            # self.debug_pub.publish(debug_msg)
            debug_msg = Bool(data = True)
            self.edge_case_pub.publish(debug_msg)

            cv2.putText(image, "Only one line", (220, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            debug_msg_pict = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg_pict)
            return

        # Find the intersection of the two lines
        # y1 = k1 * x1 + b1
        # y2 = k2 * x2 + b2
        #  k1 * x + b1 = k2 * x + b2 -> (k1-k2)*x = b2 - b1 -> x = (b2-b1)/(k1 - k2)
        try:
            x_intersect = (line2_intercept - line1_intercept) / (line1_slope - line2_slope)
            y_intersect = line1_slope * x_intersect + line1_intercept

            bottom_x = 430.0 # 480 -> 460 ! --> 500 -> 460
            bottom_y = 360.0

            x_intersect = x_intersect*4/5 + bottom_x/5
            y_intersect = y_intersect*4/5 + bottom_y/5
        except:
            # x_intersect = 200 #420 -> 440
            # y_intersect = image.shape[0]/2
            # conepx.u, conepx.v = float(x_intersect), float(y_intersect)

            # self.cone_pub.publish(conepx)
            # bbox_top_left = (int(x_intersect - 10) , int(y_intersect - 10))
            # bbox_bot_right = (int(x_intersect + 10), int(y_intersect + 10))
            # cv2.rectangle(image, bbox_top_left, bbox_bot_right, (0,255,0), 2)
            # # cv2.rectangle(image, box1_pt1, box1_pt2, (0,0,255), 2)
            # # cv2.rectangle(image, box2_pt1, box2_pt2, (0,0,255), 2)
            # debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
            # self.debug_pub.publish(debug_msg)
            debug_msg = Bool(data = True)
            self.edge_case_pub.publish(debug_msg)

            cv2.putText(image, "Only One Line", (220, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            debug_msg_pict = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.debug_pub.publish(debug_msg_pict)
            return

        # threshold_straight = 1.2
        # if no left line
        if line1_slope <= 0 and line2_slope <= 0:
            # x_intersect = 0.0
            x_intersect = 410 #420   #440 # 410
            y_intersect = image.shape[0]/2
        # if no right line
        elif line1_slope >= 0 and line2_slope >= 0:
            x_intersect = 200       #200
            # x_intersect = image.shape[1] - 1
            y_intersect = image.shape[0]/2
        # elif line1_slope <= threshold_straight and line2_slope <= threshold_straight and line1_slope >= -threshold_straight and line2_slope >= -threshold_straight:
        #     x_intersect = 320*4/5 + 460/5
        #     self.get_logger().info("Stay in the middle")

        #To-do: Bring it closer?
        conepx.u, conepx.v = float(x_intersect), float(y_intersect)
        # conepx.u, conepx.v = float(320), float(150) # for debugging

        self.cone_pub.publish(conepx)
        # img with bounding box
        # bbox_top_left = bbox[0][0], bbox[0][1]
        # bbox_bot_right = bbox[1][0], bbox[1][1]

        # self.get_logger().info(f"x intersect: {x_intersect}")
        #To-do: The boxing dimension may be different
        bbox_top_left = (int(x_intersect - 10) , int(y_intersect - 10))
        bbox_bot_right = (int(x_intersect + 10), int(y_intersect + 10))
        cv2.rectangle(image, bbox_top_left, bbox_bot_right, (0,255,0), 2)
        # cv2.rectangle(image, box1_pt1, box1_pt2, (0,0,255), 2)
        # cv2.rectangle(image, box2_pt1, box2_pt2, (0,0,255), 2)
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)
        debug_msg = Bool(data = False)
        self.edge_case_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# Possible imnprovements:
#1. If only one positive/ or one negative, it goes in a wrong direction
