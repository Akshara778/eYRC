import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, pow, sin, cos
import time 

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.odom = self.create_subscription(Odometry, '/odom', self.moniter, 10)
        self.Laser = self.create_subscription(LaserScan, '/scan', self.navigate, 10)
        self.marker = self.create_subscription(Int8, '/marker', self.markerStatus, 10)
        self.flag = self.create_publisher(Int8, '/flag', 10)
        self.done_flag = self.create_subscription(Int8, '/flag', self.wait, 10)
        self.Twist = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_position = [-1.5339, -6.6156, 1.57] # x, y, yaw
        self.targets = [[-1.5339, -6.6156, atan2(6.6156 - 5.1, 1.5339 + 0.26)], [0.26, -5.1 , 1.57], [0.26, -1.95, 1.57], [-1.48, -0.67, -1.57], [-1.53, -5.5, -1.57], [-3.22, -0.67, 1.57], [-1.48, -0.67, -1.57], [ -1.53, -6.61, -1.57 ]]
        self.check = [False, False, False, False, False, False, False, False]
        self.target_index = 0
        self.intermediate = 0
        self.dock = 0
        self.tol = 0.05
        self.tol_pos = 0.05
        self.yaw_tolerance = 0.01 # 10 degrees in radians
        self.kp = 10.0
        self.error_p = 0.0
        self.speed = Twist()
        self.marker_point = 0
        self.timer = self.create_timer(0.02, self.timer_callback)

    def wait(self, msg : Int8):
        self.get_logger().info(str(self.dock))
        self.dock = msg.data


    def moniter(self, data : Odometry):
        self.current_position[0] = data.pose.pose.position.x
        self.current_position[1] = data.pose.pose.position.y
        self.current_position[2] = 2 * atan2(data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
        if self.target_index > 7:
            self.get_logger().info("All Targets Reached!")
            return
        
        if self.target_index == 0:
            if abs(self.current_position[2] - self.targets[self.target_index][2]) < self.yaw_tolerance:
                self.check[self.target_index] = True
                self.target_index += 1

        elif self.target_index == 1:
            if abs(self.current_position[0] -0.05 - self.targets[self.target_index][0]) < self.tol_pos and abs(self.current_position[2] - self.targets[self.target_index][2]) < self.yaw_tolerance:
                self.check[self.target_index] = True
                self.target_index += 1

        elif self.target_index == 2:
            if abs(self.current_position[1] - self.targets[self.target_index][1]) < self.tol_pos:
                self.get_logger().info("Im here")
                self.dock = 1
                self.flag.publish(Int8(data = self.dock))
                self.check[self.target_index] = True
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0
                self.Twist.publish(self.speed)
                self.timer.cancel()
                time.sleep(2)
                self.timer.reset()
                self.target_index += 1

        elif self.target_index == 4:
            if sqrt(pow(self.current_position[0] - self.targets[self.target_index][0], 2) + pow(self.current_position[1] - self.targets[self.target_index][1], 2)) < 0.2 and abs(self.current_position[2] - self.targets[self.target_index][2]) < self.yaw_tolerance:
                self.check[self.target_index] = True
                self.target_index += 1
        
        elif self.target_index == 5:
            if sqrt(pow(self.current_position[0] - self.targets[self.target_index][0], 2) + pow(self.current_position[1] - self.targets[self.target_index][1], 2)) < 0.2 and abs(self.current_position[2] - self.targets[self.target_index][2]) < self.yaw_tolerance:
                self.check[self.target_index] = True
                self.target_index += 1

        else:
            if sqrt(pow(self.current_position[0] - self.targets[self.target_index][0], 2) + pow(self.current_position[1] - self.targets[self.target_index][1], 2)) < 0.2:
                self.check[self.target_index] = True
                self.target_index += 1

    def markerStatus(self, data : Int8):
        self.marker_point = data.data

    def timer_callback(self):
        self.flag.publish(Int8(data = self.dock))
        if self.marker_point == 1:
            if self.target_index != 2:
                self.speed.linear.x = 2.2
                self.speed.angular.z = 0.0
                self.Twist.publish(self.speed)
                time.sleep(2.5)
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            self.Twist.publish(self.speed)
            self.timer.cancel()
            time.sleep(2)
            self.timer.reset()
            return
        
        self.Twist.publish(self.speed)

    def navigate(self, data : LaserScan):
        
        if self.target_index > 7:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            return
        
        self.target_yaw = self.targets[self.target_index][2]
        error = self.target_yaw - self.current_position[2]
        error = atan2(sin(error), cos(error))
        error_p = sqrt(pow(self.current_position[0] - self.targets[self.target_index][0], 2) + pow(self.current_position[1] - self.targets[self.target_index][1], 2))

        self.get_logger().info(
            f"Target {self.target_index}: {self.targets[self.target_index]}, "
            f"Current: {[round(self.current_position[0],2), round(self.current_position[1],2), round(self.current_position[2],2)]}, "
            f"Ori Error: {error:.3f}"
            f"Pos Error: {error_p:.3f}"
        )

        if self.target_index == 0:
            self.speed.linear.x = 0.0
            if abs(error) >= 0.01:
                self.speed.angular.z = self.kp * error * 5
            else:
                self.speed.angular.z = 0.0

        elif self.target_index == 1:
            if abs(self.current_position[0] -0.05  - self.targets[self.target_index][0]) >= 0.01:
                self.speed.angular.z = 0.0
                self.speed.linear.x = -(self.current_position[0] - 0.05 - self.targets[self.target_index][0]) * self.kp
                #self.speed.linear.x = 1.0

            elif abs(self.current_position[0] - 0.05 - self.targets[self.target_index][0]) < 0.1 and abs(error) >= self.yaw_tolerance:
                if abs(error) >= self.yaw_tolerance:
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = self.kp * error * 20
                elif abs(error) < self.yaw_tolerance: 
                    self.speed.linear.x = 0.0
                    self.speed.angular.z = 0.0

        elif self.target_index == 2:
            if abs(self.current_position[1] - self.targets[self.target_index][1]) >= 0.05:
                self.speed.angular.z = 0.0
                self.speed.linear.x = -(self.current_position[1] - self.targets[self.target_index][1]) * self.kp
                #self.speed.linear.x = 1.0
            else:
                self.speed.angular.z = 0.0
                self.speed.linear.x = 0.0 

        elif self.target_index == 3:
            if data.ranges[180] >= 1.3 and self.current_position[2] < 1.6 and abs(self.targets[self.target_index][0] - 0.2 - self.current_position[0]) >= self.tol and abs(error) > 1.57:
                self.get_logger().info("Hi 1")
                self.speed.angular.z = 0.0
                #self.speed.linear.x = (data.ranges[180] - 0.5) * self.kp 
                self.speed.linear.x = 1.0

            elif abs(self.targets[self.target_index][0] - 0.2 - self.current_position[0]) >= self.tol and abs(error) > 1.4:
                self.get_logger().info("Hi 2")
                self.speed.linear.x = 1.0 
                if abs(error) >= 1.58:
                    self.speed.angular.z = self.kp * abs(error - 1.58) * 5

                else:
                    self.speed.angular.z = 0.0
                    if abs(self.targets[self.target_index][0] - 0.2 - self.current_position[0]) >= self.tol:
                        self.speed.linear.x = -self.kp * (self.targets[self.target_index][0] - 0.2 - self.current_position[0])
                        #self.speed.linear.x = 1.0
                    else:
                        self.speed.linear.x = 0.0

            else:
                self.get_logger().info("Hi 3")
                self.speed.linear.x = 0.0
                if abs(error) >= self.yaw_tolerance:
                    self.speed.angular.z = self.kp * error * 5
                else: 
                    self.speed.angular.z = 0.0
                    if abs(self.targets[self.target_index][1] - self.current_position[1]) >= self.tol:
                        #self.speed.linear.x = -self.kp * (self.targets[self.target_index][1] - self.current_position[1])
                        self.speed.linear.x = 1.0
                    else:
                        self.speed.linear.x = 0.0


        elif self.target_index == 4:
            if abs(self.targets[self.target_index][1] - self.current_position[1]) >= 0.01:
                self.speed.angular.z = 0.0
                self.speed.linear.x = 1.0  
                
            else: 
                self.speed.angular.z = 0.0
                self.speed.linear.x = 0.0

        elif self.target_index == 5:
            if abs(atan2(sin(3.14 - self.current_position[2]), cos(3.14 - self.current_position[2]))) >= self.yaw_tolerance and abs(error) >= self.yaw_tolerance and abs(self.targets[self.target_index][0] - 0.18 - self.current_position[0]) >= 0.3:
                self.get_logger().info("Hi 1")
                self.speed.angular.z = self.kp * (3.14 - self.current_position[2]) * 5
                self.speed.linear.x = 0.0

            elif abs(atan2(sin(3.14 - self.current_position[2]), cos(3.14 - self.current_position[2]))) < self.yaw_tolerance and abs(self.targets[self.target_index][0] - 0.18 - self.current_position[0]) >= 0.02 and abs(error) >= self.yaw_tolerance: 
                self.get_logger().info("Hi 2")
                self.speed.angular.z = 0.0
                self.speed.linear.x = 1.0

            elif abs(self.targets[self.target_index][0] - 0.18 - self.current_position[0]) < 0.3 and abs(error) >= self.yaw_tolerance:
                self.get_logger().info("Hi 3")
                self.speed.linear.x = 0.0
                self.speed.angular.z = self.kp * error * 5

            elif abs(error) < self.yaw_tolerance and abs(self.targets[self.target_index][1] - self.current_position[1]) >= 0.01:
                self.get_logger().info("Hi 4")
                self.speed.linear.x = 1.0
                self.speed.angular.z = 0.0

            else:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.0  

        elif self.target_index == 6:
            if data.ranges[180] >= 1.5 and self.current_position[2] < 1.6 and abs(self.targets[self.target_index][0] + 0.2 - self.current_position[0]) >= self.tol and abs(error) > 1.57:
                self.get_logger().info("Hi 1")
                self.speed.angular.z = 0.0
                self.speed.linear.x = 1.0

            elif abs(self.targets[self.target_index][0] + 0.2 - self.current_position[0]) >= self.tol and abs(error) > 1.4:
                self.get_logger().info("Hi 2")
                self.speed.linear.x = 1.0 
                if abs(error) >= 0:
                    self.speed.angular.z = self.kp * error * 5

                else:
                    self.speed.angular.z = 0.0
                    if abs(self.targets[self.target_index][0] + 0.2 - self.current_position[0]) >= self.tol:
                        self.speed.linear.x = -self.kp * (self.targets[self.target_index][0] + 0.2 - self.current_position[0])
                        #self.speed.linear.x = 1.0
                    else:
                        self.speed.linear.x = 0.0

            else:
                self.get_logger().info("Hi 3")
                self.speed.linear.x = 0.0
                if abs(error) >= self.yaw_tolerance:
                    self.speed.angular.z = self.kp * error * 5
                else: 
                    self.speed.angular.z = 0.0
                    if abs(self.targets[self.target_index][1] - self.current_position[1]) >= self.tol:
                        #self.speed.linear.x = -self.kp * (self.targets[self.target_index][1] - self.current_position[1])
                        self.speed.linear.x = 1.0
                    else:
                        self.speed.linear.x = 0.0

        elif self.target_index == 7:
            if abs(self.targets[self.target_index][1] - self.current_position[1]) >= 0.01:
                self.speed.angular.z = 0.0
                self.speed.linear.x = 1.0  
                
            else: 
                self.speed.angular.z = 0.0
                self.speed.linear.x = 0.0  
                
        else:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0

rclpy.init()
Node = Navigation()
rclpy.spin(Node)