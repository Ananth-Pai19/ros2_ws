#!/usr/bin/env python3

import rclpy as r
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

'''
This is the code that takes the YOLO model and sends the required info on the YOLO topic
IDK why but we decided that this would be better
This has to be run on both manual and autonomous parts of the mission
<<<<<<< This is just in case, cause orin might not be able to handle >>>>>>>
'''

class ConeDetection(Node):
    def __init__(self):
        self.get_logger().info("Initializing Cone Detection.....")
        super().__init__("cone_detection_node")

        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.color_image_sub = self.create_subscription(Image, "/zed/left/image", self.color_callback, self.qos)
        self.state_sub = self.create_subscription(Bool, "/state", self.state_callback, self.qos)

        self.yolo_msg_pub = self.create_publisher(Float32MultiArray, "/cone_bbox", self.qos)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.color_image = None
        self.model = YOLO("yolov8n.pt")
        self.bridge = CvBridge()
        self.state = False

    
    def state_callback(self, state: Bool):
        self.state = state.data

    
    def color_callback(self, color_image: Image):
        if color_image is None:
            self.get_logger().warn("My guy ZED is still not publishing color image")
            return
        
        try:
            image = self.bridge.imgmsg_to_cv2(color_image, desired_encoding="bgr8")

            results = self.model(image)

            # We now have all the bounding boxes and depending on the state we will perform different operations
            if not self.state:
                # We are in manual mode
                # In manual mode, we shall always take the cone of largest dimension in the view since we will be moving towards an object
                # Seems to me that we have to log the color of the cone only when the operator clicks on A, so we will know that 


        except Exception as e:
            self.get_logger().error(f"Exception: {e}")


    def timer_callback(self):
        # Alright now its time to do all that bs
        
        # Now the main thing is this code will be running in two stages, one when it is in manual mode
        # And the other when it is in autonomous mode
        if not self.state:
            # Manual mode
            pass
            # In manual mode, we need to keep searching for the cone that the operator is going towards
            # Hence it will be the biggest 


def main(args=None):
    r.init(args=args)
    node = ConeDetection()
    try:
        r.spin(node)
    except KeyboardInterrupt as e:
        print(f"YOU DARE {e} ME!")
    finally:
        r.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()
    
    