#!/usr/bin/env python3
import rclpy as r
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int8, Bool, Float32MultiArray
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import csv
import numpy as np
import utm
import math
from cv_bridge import CvBridge
from time import sleep

class AutoControlNode(Node):
    def __init__(self):
        super().__init__("auto_control_node")

        # Assuming that gps coords will be given in 
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.vel_pub = self.create_publisher(Twist, "/motion", self.qos)
        self.rotin_pub = self.create_publisher(Int8, "/rotin", self.qos)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, self.qos)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps_fix", self.gps_callback, self.qos)
        self.yolo_sub = self.create_subscription(Float32MultiArray, "/yolo_topic", self.yolo_callback, self.qos)
        self.state_sub = self.create_subscription(Bool, "/state", self.state_callback, self.qos)
        self.depth_sub = self.create_subscription(Image, "/zed/depth/image_rect_raw", self.depth_callback, self.qos)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.gps_ready = False
        self.depth_image = None
        self.bounding_box_data = []
        self.state = False
        self.theta = 0.0
        self.dist_to_goal = 0.0
        self.prev_distance = 0.0
        self.Kp_ang = 10                            # Remember to tune this value
        self.center_align_threshold = 5             # 5 Pixels of threshold
        self.orientation_threshold = 0.1            # Gotta tweak
        self.goal_thresh = 0.5

        self.approx_gps_file = "pallu.csv"
        # Assuming pallu.csv is of the form |Pickup Latitude|Pickup Longitude|Delivery Latitude|Delivery Longitude

        self.current_x = None                       # In local frame
        self.current_y = None 

        self.yolo_msg = None
        self.step_distance = 1.0
        self.angle_check_slope = 0.0
        self.current_distance = 0.0
        self.loaf_velocity = 30.0

        self.start_x = None                         # In global (UTM) frame
        self.start_y = None 

        self.goal_x = None                          #Initially in global frame after converting delivery gps coords through UTM; later we do shifting of origin to get it in local frame
        self.goal_y = None

        self.aligned_rover = False
        self.omega_until_cone_seen = 10.0
        self.reached_approx_gps = False
        self.captured_starting_coords = False
        self.image_width = 1280                     # Check once 
        self.dist_thresh = 1.0
        self.bridge = CvBridge()
        self.get_logger().info("Initialized")


    def state_callback(self, state: Bool):
        if state is None:
            return
        
        self.state = state.data


    def odom_callback(self, odom: Odometry):
        if odom is not None:                        # From this we can get our current position in local odom frame
            self.current_x = odom.pose.pose.position.x
            self.current_y = odom.pose.pose.position.y
            self.current_orientation = odom.pose.pose.orientation.z

    
    def depth_callback(self, depth_image: Image):
        if depth_image is None:
            self.get_logger().warn("ZED is not giving depth image")
            return
        
        try: 
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Exception in getting depth image: {e}")

    
    def gps_callback(self, gps: NavSatFix):
        if gps is None:
            self.get_logger().warn("GPS is None.....")
            return 

        if not self.captured_starting_coords:
            self.start_x , self.start_y,_,_ = utm.from_latlon(gps.latitude, gps.longitude) 
            self.captured_starting_coords = True    # This block of code NEEDS CHECKING cuz of GPS error

        self.gps_ready = True

    
    def yolo_callback(self, msg: Float32MultiArray):
        self.yolo_msg = msg

        if msg is None or self.state:
            self.yolo_msg = None
            self.get_logger().info("YOLO topic is still not publishing") 
            return
        
        bounding_box_data = msg.data

        if len(bounding_box_data) < 4:
            return
        
        self.bounding_box_data = bounding_box_data
        

    def get_depth(self):
        # Get the depth from the detected bounding box
        if self.depth_image is None:
            return

        x_center, y_center = int(self.bounding_box_data[0]), int(self.bounding_box_data[1])

        # Make sure (x,y) pixel is valid
        if (y_center < 2 or y_center >= self.depth_image.shape[0] - 2 or
            x_center < 2 or x_center >= self.depth_image.shape[1] - 2):
            return 

        # Extract a small patch (5x5) around the cone center for a stable median depth
        patch = self.depth_image[y_center - 2:y_center + 3, x_center - 2:x_center + 3].flatten()

        # Filter out invalid values (NaN, 0, or inf)
        patch = patch[np.isfinite(patch)]
        patch = patch[patch > 0]

        if len(patch) == 0:
            return 

        depth = float(np.median(patch))  # median depth (meters)
        return depth
    

    def align_rover(self):
        # X axis offset of the object
        # If the object is on the left of the central line, then positive velocity
        object_offset = (self.image_width / 2) - self.bounding_box_data[0]

        # Make the angular velocity to rotate by
        self.get_logger().info("Sending rotate command to rover to align with the cone")
        if abs(object_offset) > self.center_align_threshold:
            vel = Twist()
            vel.angular.z = self.Kp_ang * object_offset 
            self.vel_pub.publish(vel)
        else:
            self.aligned_rover = True


    def read_from_csv(self):
        with open(self.approx_gps_file, 'r') as f:
            reader = csv.DictReader(f)
            self.csv_data = list(reader)
        
        if not self.csv_data:
            self.get_logger().warn('No CSV data loaded')
            return
        
        if self.start_x is None or self.start_y is None:
            return
        
        if self.current_x is None or self.current_y is None:
            return

        # Get the gps of the closest object
        goal_gps = self.get_closest_object()

        # Convert to utm frame
        self.goal_x, self.goal_y, _, _ = utm.from_latlon(goal_gps[0], goal_gps[1]) 

        # Goal relative to the starting position
        self.goal_x = self.goal_x - self.start_x 
        self.goal_y = self.goal_y - self.start_y
        # Make the starting point the origin
        self.start_x = 0.0
        self.start_y = 0.0

        self.goal_x = self.goal_x + self.current_x  # Shifting of origin; now the goal is in local frame
        self.goal_y = self.goal_y + self.current_y 

        self.curent_distance = math.hypot(self.current_x - self.goal_x, self.current_y - self.goal_y)
        return self.goal_x, self.goal_y
    

    def get_closest_object(self):
        if self.current_x is None or self.current_y is None:
            return [0, 0]
        
        curr_distance = float('inf')
        closest_object = {}
        for row in self.csv_data:
            distance = math.dist([self.current_x, self.current_y], [float(row["odom_x"]), float(row["odom_y"])])
            if distance < curr_distance:
                curr_distance = distance
                closest_object = row

        return [float(closest_object["Pickup Latitude"]), float(closest_object["Pickup Longitude"])]
    

    def proceed_to_goal(self):
        pass


    def timer_callback(self):
        # Main loop of the mission
        # Start off only when the rover is autnomous mode
        if not self.state:
            self.get_logger().warn("Rover is in manual mode, delivery operation is performed only in autonomous mode")
            return
        
        if not self.gps_ready:
            self.get_logger().warn("Waiting for GPS...")
            return

        self.goal_x, self.goal_y = self.read_from_csv()   # Goes through reading CSV function to get required delivery positions in local frame
        # CSV is continuously being read, gotta change

        self.get_logger().info(f"Distance to goal: {self.current_distance:.2f} m")
        
        # Stop if within threshold
        # if not self.reached_approx_gps:
        #     if self.dist_to_goal <= self.dist_thresh:
        #         self.stop_robot()
        #         self.get_logger().info("Reached GPS coordinates given!")
        #         self.reached_approx_gps = True
        #     # Recalculate heading after moving step_distance
        #     elif dist_from_start - self.prev_distance > self.step_distance:
        #         a = self.dist_to_goal
        #         b = dist_from_start
        #         c = self.original_distance
        #         if b != 0 and a != 0:
        #             try:
        #                 if self.current_y - self.angle_check_slope*self.current_x >=0:
        #                     self.theta = -math.acos((a**2 + b**2 - c**2) / (2 * a * b))
        #                 else:
        #                     self.theta = math.acos((a**2 + b**2 - c**2) / (2 * a * b))
        #             except ValueError:
        #                 self.theta = 0.0
        #         self.prev_distance = b
        #         self.get_logger().info(f"Correction angle: {math.degrees(self.theta):.2f}°")
        #     else:
        #         # Apply correction
        #         if abs(self.theta) > 0.5:
        #             vel = Twist()
        #             vel.linear.x = 0.0
        #             vel.angular.z = 15.0
        #             self.vel_pub.publish(vel)
        #         else:
        #             # Move forward
        #             vel = Twist()
        #             vel.linear.x = 15.0
        #             vel.angular.z = 0.0
        #             self.vel_pub.publish(vel)
        # else:

        # Now we begin our way to the goal
        # First we need to make our way towards the gps location
        reached_goal = self.proceed_to_goal()

        if reached_goal:
            # Once we are at the goal location, we need to rotate in place until we find the cone
            # First check if in fact YOLO has detected the cone
            if self.yolo_msg is None:
                # Seems like YOLO still hasnt detected
                # Proceed on by rotating in place
                self.rotin_pub.publish(1)       # Aligns the wheels for rotinplace motion
                sleep(0.5)

                # Rotate until cone comes in sight
                self.get_logger().info("Ghumar Ghumar Ghumar Ghumar Ghume re!")
                vel = Twist()
                vel.angular.z = 25 
                self.vel_pub.publish(vel)

            else:
                # My boy finally found the cone 
                # Go towards the cone
                self.get_logger().info("YOLO has detected something...")
                self.vel_pub.publish(Twist())
                self.rotin_pub.publish(2)       # Align wheels straght
                sleep(0.5)
                self.rotin_pub.publish(0)

                # Align the rover with the cone
                if not self.aligned_rover:
                    self.align_rover()           

                self.get_logger().info("Rover is aligned and facing the cone....")

                # Get the depth of the object
                depth = self.get_depth()

                if depth is None:
                    raise TypeError("For some bizarre reason, depth is None")
                
                # Proceed onwards
                vel = Twist()
                vel.linear.x = self.loaf_velocity

                # If while going to the cone we deflect, set aligned_rover to False
                if abs(self.bounding_box_data[0] - (self.image_width / 2)) > self.center_align_threshold * 5:
                    self.aligned_rover = False 

                if depth < self.goal_thresh:
                    self.get_logger().info("Reached delivery location!") # Operator should press A for manual mode over here
                    self.vel_pub.publish(Twist())
                    return
                
                self.vel_pub.publish(vel)


        # vel = Twist()
        # if self.yolo_msg is None:
        #     orientation_diff = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x) - self.current_orientation
        #     if abs(orientation_diff) > self.orientation_threshold:
        #         vel.angular.z = self.Kp_ang * orientation_diff
        #         vel.linear.x = 0.0                  # Many logic issues are there here, gotta discuss
        #     else:
        #         vel.angular.z = 0.0
        #         vel.linear.x = self.loaf_velocity
        #     self.vel_pub.publish(vel)

        # else:
        #     if not self.aligned_rover:
        #         self.align_rover() 
        #     self.get_logger().info("Rover is aligned and facing the cone....")
        #     # Get the depth of the object
        #     depth = self.get_depth()

        #     if depth is None:
        #         raise TypeError("For some bizarre reason, depth is None")
            
        #     # Proceed onwards
        #     vel.linear.x = self.loaf_velocity
        #     vel.angular.z = 0.0


        #     # If while going to the cone we deflect, set aligned_rover to False
        #     if abs(self.bounding_box_data[0] - (self.image_width / 2)) > self.center_align_threshold * 5:
        #         self.aligned_rover = False 

        #     if depth < self.dist_thresh:
        #         self.get_logger().info("Reached delivery location!") # Operator should press A for manual mode over here
        #         vel.linear.x = 0.0
        #         vel.angular.z = 0.0
        #         self.vel_pub.publish(vel)
        #         return
            
        #     self.vel_pub.publish(vel)

                # if self.distance < 1.0:
                #     self.get_logger().info("Reached delivery location!") # Operator should press A for manual mode over here
                #     with open(self.odom_csv_file, 'w', newline='') as f:
                #         if self.csv_data:  # Check if there's any data
                #             writer = csv.DictWriter(f, fieldnames=self.reader.fieldnames)
                #             writer.writeheader()
                #             # Skip the current, just finished data row, write the rest
                #             writer.writerows(self.csv_data[:self.row_num])
                #             if self.row_num < len(self.csv_data - 1):
                #                 writer.writerows(self.csv_data[self.row_num+1:])
                #         else:
                #             self.get_logger().warn('No CSV data to write')
                            
                #     return
        # Now that we are in autonomous mode, the operator has in fact moved to any object
        # So we need to get its corresponding delivery location from the pallu.csv

    # def stop_robot(self):
    #     vel = Twist()
    #     vel.linear.x = 0.0
    #     vel.angular.z = 0.0
    #     self.vel_pub.publish(vel)
            
    
def main(args=None):
    r.init(args=args)
    node = AutoControlNode()
    try:
        r.spin(node)
    except KeyboardInterrupt as e:
        print(f"YOU DARE {e} ME!")
    finally:
        r.shutdown()
        node.destroy_node()

if __name__ == "__main__":
    main()