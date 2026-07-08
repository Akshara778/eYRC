import rclpy
import time
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
from math import atan2, sqrt, sin, cos

class LidarProcessor(Node):
    def __init__(self):
        super().__init__("lidar_processor")
        self.tol_ang = 0.07
        self.tol_pos = 0.5
        self.tol = 0.3
        self.subscriber = self.create_subscription(Odometry, "/odom", self.monitor, 10)
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.obstacle = self.create_subscription(LaserScan, "/scan", self.move, 10)
        self.check_marker = self.create_subscription(Int8, "/marker", self.marker_stop, 10)
        self.check = [0, 0, 0]
        self.current_target = 0
        self.target_pos = [[-1.53, -1.95, 1.57], [0.13, 1.24, 0.00], [0.38, -3.32, -1.57]]
        self.speed = Twist()
        self.x = -1.5339
        self.y = -6.6156
        self.theta = 1.57
        self.marker = 0

    def marker_stop(self, msg : Int8):
        self.marker = msg.data

    def monitor(self, current_pos : Odometry):
        self.x = current_pos.pose.pose.position.x
        self.y = current_pos.pose.pose.position.y
        self.theta = 2 * atan2(current_pos.pose.pose.orientation.z, current_pos.pose.pose.orientation.w)
        if self.current_target > 2:
            return
        if sqrt((self.x - self.target_pos[self.current_target][0])**2 + (self.y - self.target_pos[self.current_target][1])**2) < self.tol and abs(self.theta - self.target_pos[self.current_target][2]) < 0.3:
            self.check[self.current_target] = 1
            self.current_target += 1

    def timer_callback(self):
        self.publisher.publish(self.speed)

    def move(self, data : LaserScan):
        if self.marker == 1:
            self.timer.cancel()
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            self.publisher.publish(self.speed)
            time.sleep(2)
            self.timer.reset()
        if self.current_target > 2:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0
            return
        
        self.target_angle = self.target_pos[self.current_target][2]
        error = self.target_angle - self.theta
        error = atan2(sin(error), cos(error))

        if self.current_target == 0:
            self.speed.linear.x = 1.0
        elif self.current_target == 1 and abs(self.target_pos[self.current_target][1] + 0.4 - self.y) >= self.tol_pos:
            self.speed.linear.x = 0.7 * (self.target_pos[self.current_target][1] + 0.4 - self.y)
        elif self.current_target == 2:
            if abs(self.target_pos[self.current_target][0] + 0.3 - self.x) >= self.tol_pos:
                self.speed.linear.x = 0.7 * (self.target_pos[self.current_target][0] + 0.3 - self.x)
            elif abs(error) >= self.tol_ang:
                self.speed.linear.x = 0.0
                self.speed.angular.z = 0.7 * error
            else:
                self.speed.linear.x = -0.7 * (self.target_pos[self.current_target][1] - self.y)
        else:
            self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
            

        if self.speed.linear.x == 0 and abs(error) >= self.tol_ang:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.7 * error
        
        elif self.speed.linear.x == 0 and abs(error) < self.tol_ang:
            self.speed.linear.x = 0.7 * (self.target_pos[self.current_target][0] - self.x)
            self.speed.angular.z = 0.0
        
        if data.ranges[180] < 0.3:
            self.speed.linear.x = 0.0
            self.speed.angular.z = 0.0


rclpy.init()
node = LidarProcessor()
rclpy.spin(node)
rclpy.shutdown()
