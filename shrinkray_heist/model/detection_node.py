import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from vs_msgs.msg import ConeLocationPixel

from .detector import Detector
##NEED TEST AGAIN
class DetectorNode(Node):
    def __init__(self):
        super().__init__("detection_node")
        self.detector = Detector()
        self.debug_pub = self.create_publisher(Image, "/banana_debug_img", 10)
        self.publisher = self.create_publisher(ConeLocationPixel, "/relative_banana_px", 10)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.state_sub = None #implement with Audreys Node
        self.odom_sub =  None #might want to know where we are
        self.valid_state = True
        self.bridge = CvBridge()

        self.get_logger().info("Detector Initialized")

    def state_callback(self, statemsg):
        """
        Used to determine when we should run inference with camera. If the 
        state is a valid state (when we could potentiatlly see banana)
        then we should try to detect banana
        """
        self.state_valid = True

    def callback(self, img_msg):
        if self.state_valid is True:
            # Process image with CV Bridge
            image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
            model_out = self.dectector.predict(image)
            debug_img = model_out
            predictions =  model_out["predictions"]
            bananas = [p for p in predictions if p[-1] == "banana"]
            if bananas:
                cone_px = ConeLocationPixel
                banana_coords = bananas[0][:-1]
                bottomv = float(banana_coords[3])
                bottomu = float(banana_coords[0] + bbox[2])//2
                cone_px.u, cone_px.v = bottomu, bottomv
                self.publisher.publish(cone_px)

                original_image = model_out["original_image"]
                out = model.draw_box(original_image, bananas, draw_all=True)
                debug_msg = self.bridge.cv2_to_imgmsg(np.array(out), "bgr8")
                self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
