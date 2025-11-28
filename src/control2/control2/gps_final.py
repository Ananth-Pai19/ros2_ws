#!/usr/bin/env python3
import rclpy as r
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
import math
import utm 

class Drive(Node):

    def __init__(self):
        super().__init__("drive_node")

        # ====== Direct GPS navigation variables ======
        self.start_lat = None
        self.start_lon = None
        self.start_x = 0.0
        self.start_y = 0.0
        self.goal_lat = 12.9916
        self.goal_lon = 80.2338
        self.goal_x, self.goal_y, _, _ = utm.from_latlon(self.goal_lat, self.goal_lon)
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.original_distance = 0.0
        self.gps_ready = False
        self.angle_check_slope = 0.0
        self.prev_distance = 0.0
        self.step_distance = 1.0
        self.tolerance = 0.5
        self.theta = 0.0
        self.prev_lon = 0.0
        self.prev_lat = 0.0
        self.control_loop_start = False
        self.heading_angle = None

        self.vel_pub = self.create_publisher(Twist, "/motion", 10)
        self.gps_sub = self.create_subscription(NavSatFix, "/sim/fix_gps", self.gps_callback, 10)
        self.create_timer(0.1, self.control_loop)


    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            self.get_logger().warn("No GPS fix yet...")
            return
        
        self.curr_x, self.curr_y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        # #Finding the heading angle
        # if self.heading_angle == None:
        #     self.curr_x, self.curr_y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)
        #     vel = Twist()
        #     vel.linear.x = 15.0
        #     vel.angular.z = 0.0
        #     self.vel_pub.publish(vel)
        #     if math.hypot(self.curr_x - self.start_x, self.curr_y - self.start_y) > 1.0:
        #         heading_angle = math.atan2(
        #             math.sin(math.radians(msg.longitude - self.prev_lon)) * math.cos(math.radians(msg.latitude)),
        #             math.cos(math.radians(self.prev_lat)) * math.sin(math.radians(msg.latitude)) -
        #             math.sin(math.radians(self.prev_lat)) * math.cos(math.radians(msg.latitude)) *
        #             math.cos(math.radians(msg.longitude - self.prev_lon))
        #         )
        #         self.stop_robot()
        #         self.get_logger().info('The heading angle has been calculated')
        #         self.heading_angle = heading_angle
        #         if not self.control_loop_start:
        #             self.create_timer(0.1, self.control_loop)
        #             self.control_loop_start = True

        #     self.prev_lat = msg.latitude
        #     self.prev_lon= msg.longitude
        
        # else: 
        # Get the starting coords only at the beginning
        if self.start_lat is None:
            self.start_lat, self.start_lon = msg.latitude, msg.longitude
            self.start_x, self.start_y, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

        if self.goal_x != self.start_x:
            self.angle_check_slope = (self.goal_y - self.start_y)/(self.goal_x - self.start_x)

        self.original_distance = math.hypot(self.start_x - self.goal_x, self.start_y - self.goal_y)
        self.gps_ready = True
        self.get_logger().info("GPS ready.....")


    def control_loop(self):
        if not self.gps_ready:
            self.get_logger().warn("Waiting for GPS...")
            return

        # Get our distance from the goal and starting location, at any given point
        dist_to_goal = math.hypot(self.goal_x - self.curr_x, self.goal_y - self.curr_y)
        dist_from_start = self.original_distance - dist_to_goal
        self.get_logger().info(f"Distance to goal: {dist_to_goal:.2f} m")

        # Stop if within tolerance
        if dist_to_goal <= self.tolerance:
            self.vel_pub.publish(Twist())
            self.get_logger().info("Reached goal!")
            return
        
        # Recalculate heading after moving step_distance
        if dist_from_start - self.prev_distance > self.step_distance:
            # Applying some math formula, thats why converting to a,b and c
            a = dist_to_goal
            b = dist_from_start
            c = self.original_distance

            if b != 0 and a != 0:
                try:
                    theta = math.acos((a**2 + b**2 - c**2) / (2 * a * b))
                    if self.curr_y - self.angle_check_slope*self.curr_x >= 0:
                        self.theta = -theta
                    else:
                        self.theta = theta
                except ValueError:
                    self.theta = 0.0

            # Reset the previous distance
            self.prev_distance = b
            self.get_logger().info(f"Correction angle: {math.degrees(self.theta):.2f}°")
            
        # Onwards we go with the heading
        vel = Twist()
        if abs(self.theta) > 0.05:
            vel.linear.x = 0.0
            vel.angular.z = 15.0
        else:
            # Move forward
            vel.linear.x = 15.0
            vel.angular.z = 0.0
        # Send velocity command
        self.vel_pub.publish(vel)

def main(args=None):
    r.init(args=args)
    node = Drive()
    r.spin(node)
    r.shutdown()

if __name__ == "__main__":
    main()
