#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class ConeFollower(Node):

    def __init__(self):
        """
        This node does exactly what you described:
        1. Reads YOLO detections of the cone (pixel coords).
        2. Reads ZED camera depth image.
        3. Rotates rover until cone center is at image center (x direction).
        4. Then drives straight until the cone is close enough.
        Everything is done in pixel space — no need for fx, cx, or coordinate transforms.
        """

        super().__init__('cone_follower_node')

        # ------------- CAMERA PARAMETERS -------------
        # Define your image resolution (ZED RGB/depth image size)
        self.image_width = 1280
        self.image_height = 720
        self.angle_field_of_view = 180
        self.angle_to_turn = 0.0
        self.angle_found = False
        self.initial_yaw = 0.0
        self.initial_yaw_found = False
        self.depth = 0.0
        self.x_err = 0.0
        # Compute image center — our goal is to align cone x_center to this
        self.image_center_x = self.image_width // 2
        self.angle_limit = 0.1

        self.pixel_tolerance = 40      # pixels (when |x_err| < this, we consider cone centered)
        self.stop_distance = 0.8       # meters (stop when cone is this close)
        self.k_ang = 0.005             # angular velocity gain (rotation speed per pixel offset)
        self.k_lin = 0.4               # linear velocity gain (forward speed)
        self.max_ang = 0.8             # max angular velocity (rad/s)
        self.max_lin = 0.6             # max linear velocity (m/s)

        self.bridge = CvBridge()       # converts ROS Image -> OpenCV
        self.depth_image = None        # stores latest ZED depth frame
        self.target_bbox = None        # stores latest YOLO detection [x_center, y_center, width, height, conf]
        self.state = "ROTATE"          # 'ROTATE' -> 'FORWARD' -> 'STOP'

        self.depth_sub = self.create_subscription(Image, "/zed/depth/image_rect_raw", self.depth_callback, 10)

        # Subscribe to YOLO bounding box center (Float32MultiArray [x_center, y_center, width, height, conf])
        self.yolo_sub = self.create_subscription(Float32MultiArray, "/cone_bbox", self.bbox_callback, 10)

        # Publish velocity commands to /motion (Float32MultiArray [linear_vel, angular_vel])
        # The drive node already listens to this topic.
        self.motion_pub = self.create_publisher(Twist, "/motion", 10)
        
        # To run based on angle: backup in case pixel fails
        #self.create_subscription(Odometry, "zed/odom", self.odom_callback, 5) #put correct topic for odometry

        # Run control loop at 20 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Pixel-based ConeFollower node started!")


    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth conversion error: {e}")


    def bbox_callback(self, msg):
        if len(msg.data) < 5:
            return
        x, y, w, h, conf = msg.data
        if conf > 0.8:  # skip low-confidence detections
            self.target_bbox = (int(x), int(y))
        else:
            self.target_bbox = None
            self.angle_found = False
            self.initial_yaw_found = False


    def control_loop(self):
        if self.target_bbox is None or self.depth_image is None:
            return

        x_center, y_center = self.target_bbox

        # Make sure (x,y) pixel is valid
        if (y_center < 2 or y_center >= self.depth_image.shape[0] - 2 or
            x_center < 2 or x_center >= self.depth_image.shape[1] - 2):
            return

        # Extract a small patch (5x5) around the cone center for a stable median depth
        patch = self.depth_image[y_center-2:y_center+3, x_center-2:x_center+3].flatten()

        # Filter out invalid values (NaN, 0, or inf)
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]
        if len(patch) == 0:
            return

        depth = float(np.median(patch))  # median depth (meters)
        self.depth = depth

        # Positive if cone is to the RIGHT of image center
        # Negative if cone is to the LEFT
        x_err = x_center - self.image_center_x
        self.x_err = x_err
        
        twist_message = Twist()
        
        if abs(x_err) > self.pixel_tolerance:
            self.state = "ROTATE"

        if self.state == "ROTATE":
            # If cone is not centered horizontally
            if abs(x_err) > self.pixel_tolerance:
                # Rotate proportionally to pixel error
                omega = -self.k_ang * x_err   # negative sign: right cone → rotate right
                omega = max(-self.max_ang, min(self.max_ang, omega))
                vel = 0.0
                self.get_logger().info(f" Rotating | x_err={x_err:.1f}px omega={omega:.2f}")
            else:
                # Cone roughly centered → switch to forward state
                self.state = "FORWARD"
                vel = 0.0
                omega = 0.0
                self.get_logger().info(" Cone centered, switching to FORWARD mode.")

        elif self.state == "FORWARD":
            # Move forward until depth (distance to cone) <= stop threshold
            if depth > self.stop_distance:
                vel = self.k_lin * (depth - self.stop_distance)
                vel = min(self.max_lin, vel)
                omega = 0.0
                self.get_logger().info(f" Moving forward | depth={depth:.2f} m vel={vel:.2f}")
            else:
                self.state = "STOP"
                vel = 0.0
                omega = 0.0
                self.get_logger().info(" Stopping — reached cone.")

        else:  # STOP
            vel = 0.0
            omega = 0.0
            
        twist_message.linear.x = vel
        twist_message.angular.z = omega
        self.motion_pub.publish(twist_message)


def main(args=None):
    rclpy.init(args=args)
    node = ConeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
