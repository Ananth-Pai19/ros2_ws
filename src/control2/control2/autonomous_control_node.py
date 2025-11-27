#!/usr/bin/env python3
import rclpy as r
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8, Bool
from nav_msgs.msg import Odometry
import csv
from time import sleep

class AutonomousControl(Node):
    def __init__(self):
        self.qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.twist_pub = self.create_publisher(Twist, "/motion", self.qos)
        self.rot_pub = self.create_publisher(Int8, "/rot", self.qos)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, self.qos)
        self.yolo_sub = self.create_subscription(Int8, "/yolo_topic", self.yolo_callback, self.qos)
        self.state_sub = self.create_subscription(Bool, "/state", self.state_callback, self.qos)

        # Define the csv stuff, we will read from kavin.csv
        self.odom_csv_file = "kavin.csv"
        self.current_object_number = 1
        self.error_thresh = 0.2
        self.Kp = 15                                # Remember to tune this value
        self.loaf_velocity = 30                     # Remember to change this value
        self.yolo_msg = None
        self.goal_thresh = 0.3

        self.current_orientation = 0
        self.current_x = 0
        self.current_y = 0                    
        # Make a timer as the control loop of the main program  
        self.timer = self.create_timer(0.1, self.timer_callback)
        

    def state_callback(self, state: Bool):
        self.state = state.data


    def yolo_callback(self, msg: Int8):
        self.yolo_msg = msg


    def odom_callback(self, odom: Odometry):
        # From this we can get our current position, or as required by your guys go-to-goal logic
        if odom is not None:
            self.current_orientaion = odom.pose.pose.orientation.z


    def read_from_csv(self):
        with open(self.odom_csv_file, 'r') as f:
            reader = csv.DictReader(f)
            self.csv_data = list(reader)

        if not self.csv_data:
            self.get_logger().warn('No CSV data loaded')
            return 
        
        if self.current_object_number < 1 or self.current_object_number >= len(self.csv_data):
            self.get_logger().warn(f'Counter {self.current_object_number} out of range (0-{len(self.csv_data)-1})')
            return 
        
        row = self.csv_data[self.current_object_number]
        orientation = float(row[2])
        return orientation
    
        
    def timer_callback(self):
        # Main loop begins when the operator first clicks the A button
        # Now we begin our journey towards the delivery location of the corresponding cone

        # First check if the rover is in autonomous mode or not
        if not self.state:
            self.get_logger().warn("Rover is in manual mode, delivery operation will begin in autonomous mode")
            return

        orientation = self.read_from_csv()
        
        if orientation is None:
            return
        # Having gotten the orientation, we shall rotate until we are at this orientation only
        twist_message = Twist()

        if abs((error := (orientation - self.current_orientation))) > self.error_thresh:
            # Orientation is angle after all
            # Error is greater than our threshold, so perform the rotate-in-place
            self.rot_pub.publish(1)  # -> Makes the rover steer to rotate in place
            sleep(2)                 # Sleeping for 2 seconds, assuming it takes this much time to align its wheels
            
            # Could use a PID controller here, but P is fine for now
            self.rotate_in_place_velocity = self.Kp * error

            # Publish this autonomous omega now
            twist_message.angular.z = self.rotate_in_place_velocity
            self.twist_pub.publish(twist_message)

        else:
            # Alright now i think we are facing the cone
            self.rot_pub.publish(2)     # -> Aligns rover straight
            sleep(2)                    # Assuming it takes 2s to align
            self.rot_pub.publish(0)
            
            # Rover is facing the right direction now, so now we proceed in this direction
            if self.yolo_msg is None:
                twist_message.linear.x = self.loaf_velocity
                self.twist_pub.publish(twist_message)
                # Loaf velocity is only passed when yolo hasnt detected
            else:
                # The yolo model has detected and all that, need the code to understand
                # Some nonsense here
                ###############################
                # if goal reached
                self.get_logger().info("Reached delivery location, waiting for operator to drop.....")

                # Onwards to next waypoint
                self.current_object_number += 1
                pass
                
            # Goal reached checking shall be done when yolo has started its detection
            # Once reached publish zero velocity and keep printing "Waiting for operator to pick object up...."


def main(args=None):
    r.init(args=args)
    node = AutonomousControl()
    try:
        r.spin(node)
    except KeyboardInterrupt:
        print("Interrupted")
    finally:
        r.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
    