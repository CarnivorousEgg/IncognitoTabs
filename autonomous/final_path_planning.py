import math
import rclpy 
from time import sleep 
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import Twist     
from sensor_msgs.msg import LaserScan    
from geometry_msgs.msg import Pose 
from std_msgs.msg import Float64MultiArray
from rclpy.qos import qos_profile_sensor_data 
import numpy as np 
class PlaceholderController(Node):
 
    def __init__(self):
        super().__init__('PlaceholderController')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/state_est',
            self.state_estimate_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
             
        self.subscription_goal_pose = self.create_subscription(
            Pose,
            '/goal',
            self.pose_received,
            10)
             
        self.publisher_ = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10)
        self.left_dist = 999999.9 # Left
        self.leftfront_dist = 999999.9 # Left-front
        self.front_dist = 999999.9 # Front
        self.rightfront_dist = 999999.9 # Right-front
        self.right_dist = 999999.9 # Right
        self.forward_speed = 0.035 
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.robot_mode = "go to goal mode"
        self.dist_thresh_obs = 0.25 # in meters
        self.turning_speed = 0.25 # rad/s
        self.go_to_goal_state = "adjust heading"
        self.goal_x_coordinates = False # [ 0.0, 3.0, 0.0, -1.5, -1.5,  4.5, 0.0]
        self.goal_y_coordinates = False # [-4.0, 1.0, 1.5,  1.0, -3.0, -4.0, 0.0]
        self.goal_idx = 0
        self.goal_max_idx =  None # len(self.goal_x_coordinates) - 1 
         
        self.yaw_precision = 2.0 * (math.pi / 180) 
        self.turning_speed_yaw_adjustment = 0.0625
         
        self.dist_precision = 0.2
        self.wall_following_state = "turn left"
        self.turning_speed_wf_fast = 1.0  # Fast turn
        self.turning_speed_wf_slow = 0.125 # Slow turn
         
        self.dist_thresh_wf = 0.45 # in meters  
        self.dist_too_close_to_wall = 0.15 # in meters
        self.bug2_switch = "ON"
         
        self.start_goal_line_calculated = False
        self.start_goal_line_slope_m = 0
        self.start_goal_line_y_intercept = 0
        self.start_goal_line_xstart = 0
        self.start_goal_line_xgoal = 0
        self.start_goal_line_ystart = 0
        self.start_goal_line_ygoal = 0
         
        self.dist_thresh_bug2 = 0.15
         
        self.distance_to_start_goal_line_precision = 0.1
        self.hit_point_x = 0
        self.hit_point_y = 0
         
        self.distance_to_goal_from_hit_point = 0.0
         
        self.leave_point_x = 0
        self.leave_point_y = 0
         
        self.distance_to_goal_from_leave_point = 0.0
         
        self.leave_point_to_hit_point_diff = 0.25 # in mete

    def pose_received(self,msg):
        self.goal_x_coordinates = [msg.position.x]
        self.goal_y_coordinates = [msg.position.y]
        self.goal_max_idx = len(self.goal_x_coordinates) - 1    
    def scan_callback(self, msg):
        self.left_dist = msg.ranges[180]
        self.leftfront_dist = msg.ranges[135]
        self.front_dist = msg.ranges[90]
        self.rightfront_dist = msg.ranges[45]
        self.right_dist = msg.ranges[0]
             
    def state_estimate_callback(self, msg):
        curr_state = msg.data
        self.current_x = curr_state[0]
        self.current_y = curr_state[1]
        self.current_yaw = curr_state[2]
        if self.goal_x_coordinates == False and self.goal_y_coordinates == False:
            return
        if self.bug2_switch == "ON":
            self.bug2()
        else:
             
            if self.robot_mode == "go to goal mode":
                self.go_to_goal()
            elif self.robot_mode == "wall following mode":
                self.follow_wall()
            else:
                pass # Do nothing  
    def go_to_goal(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        if self.bug2_switch == "ON":
            d = self.dist_thresh_bug2
            if (    self.leftfront_dist < d or
                self.front_dist < d or
                self.rightfront_dist < d):
                self.robot_mode = "wall following mode"
                self.hit_point_x = self.current_x
                self.hit_point_y = self.current_y
                self.distance_to_goal_from_hit_point = (
                    math.sqrt((
                    pow(self.goal_x_coordinates[self.goal_idx] - self.hit_point_x, 2)) + (
                    pow(self.goal_y_coordinates[self.goal_idx] - self.hit_point_y, 2))))    
                msg.angular.z = self.turning_speed_wf_fast
                self.publisher_.publish(msg)
                return
        if (self.go_to_goal_state == "adjust heading"):
            desired_yaw = math.atan2(
                    self.goal_y_coordinates[self.goal_idx] - self.current_y,
                    self.goal_x_coordinates[self.goal_idx] - self.current_x)
            yaw_error = desired_yaw - self.current_yaw
            if math.fabs(yaw_error) > self.yaw_precision:
                if yaw_error > 0:    
                    msg.angular.z = self.turning_speed_yaw_adjustment               
                else:
                    msg.angular.z = -self.turning_speed_yaw_adjustment
                 
                self.publisher_.publish(msg)
                 
            else:               
                self.go_to_goal_state = "go straight"
                self.publisher_.publish(msg)        
        elif (self.go_to_goal_state == "go straight"):
            position_error = math.sqrt(
                        pow(
                        self.goal_x_coordinates[self.goal_idx] - self.current_x, 2)
                        + pow(
                        self.goal_y_coordinates[self.goal_idx] - self.current_y, 2)) 
            if position_error > self.dist_precision:
                msg.linear.x = self.forward_speed
                self.publisher_.publish(msg)
                desired_yaw = math.atan2(
                    self.goal_y_coordinates[self.goal_idx] - self.current_y,
                    self.goal_x_coordinates[self.goal_idx] - self.current_x)
                yaw_error = desired_yaw - self.current_yaw      
                if math.fabs(yaw_error) > self.yaw_precision:
                    self.go_to_goal_state = "adjust heading"
            else:           
                self.go_to_goal_state = "goal achieved"
                self.publisher_.publish(msg)
        elif (self.go_to_goal_state == "goal achieved"):
            self.get_logger().info('Goal achieved! X:%f Y:%f' % (
                self.goal_x_coordinates[self.goal_idx],
                self.goal_y_coordinates[self.goal_idx]))
            self.goal_idx = self.goal_idx + 1
            if (self.goal_idx > self.goal_max_idx):
                self.get_logger().info('Congratulations! All goals have been achieved.')
                while True:
                    pass
            else: 
                self.go_to_goal_state = "adjust heading"               
            self.start_goal_line_calculated = False            
        else:
            pass
    def follow_wall(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0        
        if self.bug2_switch == "ON":
            x_start_goal_line = self.current_x
            y_start_goal_line = (
                self.start_goal_line_slope_m * (
                x_start_goal_line)) + (
                self.start_goal_line_y_intercept)
            distance_to_start_goal_line = math.sqrt(pow(
                        x_start_goal_line - self.current_x, 2) + pow(
                        y_start_goal_line - self.current_y, 2)) 
            if distance_to_start_goal_line < self.distance_to_start_goal_line_precision:
                self.leave_point_x = self.current_x
                self.leave_point_y = self.current_y
                self.distance_to_goal_from_leave_point = math.sqrt(
                    pow(self.goal_x_coordinates[self.goal_idx] 
                    - self.leave_point_x, 2)
                    + pow(self.goal_y_coordinates[self.goal_idx]  
                    - self.leave_point_y, 2)) 
                diff = self.distance_to_goal_from_hit_point - self.distance_to_goal_from_leave_point
                if diff > self.leave_point_to_hit_point_diff:
                    self.robot_mode = "go to goal mode"
                return             
        d = self.dist_thresh_wf
        if self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
        elif (self.leftfront_dist > d and self.front_dist > d and self.rightfront_dist < d):
            if (self.rightfront_dist < self.dist_too_close_to_wall):
                self.wall_following_state = "turn left"
                msg.linear.x = self.forward_speed
                msg.angular.z = self.turning_speed_wf_fast      
            else:           
                self.wall_following_state = "follow wall" 
                msg.linear.x = self.forward_speed   
        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist > d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
        elif self.leftfront_dist > d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist > d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
        elif self.leftfront_dist < d and self.front_dist < d and self.rightfront_dist < d:
            self.wall_following_state = "turn left"
            msg.angular.z = self.turning_speed_wf_fast
        elif self.leftfront_dist < d and self.front_dist > d and self.rightfront_dist < d:
            self.wall_following_state = "search for wall"
            msg.linear.x = self.forward_speed
            msg.angular.z = -self.turning_speed_wf_slow # turn right to find wall
        else:
            pass
        self.publisher_.publish(msg) 
    def bug2(self):
        if self.start_goal_line_calculated == False:
            self.robot_mode = "go to goal mode"            
            self.start_goal_line_xstart = self.current_x
            self.start_goal_line_xgoal = self.goal_x_coordinates[self.goal_idx]
            self.start_goal_line_ystart = self.current_y
            self.start_goal_line_ygoal = self.goal_y_coordinates[self.goal_idx]
            self.start_goal_line_slope_m = (
                (self.start_goal_line_ygoal - self.start_goal_line_ystart) / (
                self.start_goal_line_xgoal - self.start_goal_line_xstart))
            self.start_goal_line_y_intercept = self.start_goal_line_ygoal - (
                    self.start_goal_line_slope_m * self.start_goal_line_xgoal) 
            self.start_goal_line_calculated = True
        if self.robot_mode == "go to goal mode":
            self.go_to_goal()           
        elif self.robot_mode == "wall following mode":
            self.follow_wall()
def main(args=None):
    rclpy.init(args=args)
     
    controller = PlaceholderController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()
