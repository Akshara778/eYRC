import rclpy
from rclpy.node import Node
from rclpy.publisher import Duration
from std_msgs.msg import String, Int8
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import atan2, sqrt


class ShapeDetectorNode(Node):
    def __init__(self):
        super().__init__("shape_detector")
        self.publisher = self.create_publisher(String, "/detection_status", 10)
        self.marker_publisher = self.create_publisher(Int8, "/marker", 10)
        self.timer = self.create_timer(0.1, self.marker_detection)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.scanner, 10)
        self.pos_subscriber = self.create_subscription(Odometry, "/odom", self.update_position, 10)
        self.vel_subscriber = self.create_subscription(Twist, "/cmd_vel", self.update_velocity, 10)
        self.previous_distance = [0, 0]
        self.current_distance = [0, 0]
        self.x = -1.5339
        self.y = -6.6156
        self.theta = 1.57
        self.count = 0
        self.dec_lower_threshold = 0.15
        self.dec_upper_threshold = 0.3
        self.wall_threshold = 0.4
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.angle_tol = 0.02
        self.marker_state = 0
        self.wall_distance = [0, 0]
        self.current_marker = [0, 0]
        self.pentagon = [0.26, -1.95, 1.57]
        self.flag = 0
        self.last_detection_time = None
        self.last_msg = None
        self.tray = 0
        self.direction = 0
    

    def tray_number(self):
        if self.x > 0.2:
            if self.y < -3.6875:
                self.tray = 1
            elif self.y < -2.175:
                self.tray = 2
            elif self.y < -0.6625:
                self.tray = 3
            else:
                self.tray = 4
        elif self.x > -1.65:
            if self.direction == 0:
                if self.y > -0.6625:
                    self.tray = 4
                elif self.y > -2.175:
                    self.tray = 3
                elif self.y > -3.6875:
                    self.tray = 2
                else:
                    self.tray = 1

            elif self.direction == 1:
                if self.y > -0.6625:
                    self.tray = 8
                elif self.y > -2.175:
                    self.tray = 7
                elif self.y > -3.6875:
                    self.tray = 6
                else:
                    self.tray = 5
        else:
            if self.y < -3.6875:
                self.tray = 5
            elif self.y < -2.175:
                self.tray = 6
            elif self.y < -0.6625:
                self.tray = 7
            else:
                self.tray = 8
                



    def marker_detection(self):
        state = Int8()
        state.data = self.marker_state
        self.marker_publisher.publish(state)

        if self.last_detection_time and self.last_msg:
            if (self.get_clock().now() - self.last_detection_time) < Duration(seconds = 2.0):
                if self.last_msg == "DOCK_STATION,":
                    self.publisher.publish(String(data = self.last_msg + str(self.x) + ',' + str(self.y) + ',0'))
                else: 
                    self.publisher.publish(String(data = self.last_msg + str(self.x) + ',' + str(self.y) + ',' + str(self.tray)))
            else:
                self.last_detection_time = None
                self.last_msg = None

    def update_position(self, msg : Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #self.get_logger().info(f"Position x: {self.x}, y: {self.y}")
        self.theta = 2 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        if abs(self.y - self.pentagon[1]) < 0.05 and self.flag == 0:
            self.marker_state = 1
            string = String()
            self.current_marker = [self.x, self.y]
            string.data = "DOCK_STATION," + str(self.x) + ',' + str(self.y) + ',0'
            self.publisher.publish(string)
            self.last_detection_time = self.get_clock().now()
            self.last_msg = "DOCK_STATION,"
            self.flag = 1

    def update_velocity(self, msg : Twist):
        self.linear_vel = msg.linear.x
        self.angular_vel = msg.angular.z

    def scanner(self, msg : LaserScan):
        self.tray_number()
        self.current_distance = [msg.ranges[359], msg.ranges[0]]
        if self.previous_distance == [0, 0] or (self.theta > 0 and abs(self.theta - 1.57) > self.angle_tol) or (self.theta < 0 and abs(self.theta + 1.57) > self.angle_tol) or self.linear_vel < 0.01:
            self.marker_state = 0
            self.previous_distance = self.current_distance.copy()
            return
    
        dec_left = self.previous_distance[0] - self.current_distance[0]
        dec_right = self.previous_distance[1] - self.current_distance[1]
        #self.get_logger().info(f"left: {self.wall_distance[0] - self.current_distance[0]}, right: {self.wall_distance[1] - self.current_distance[1]}, marker: {self.marker_state}")

        if dec_left > self.wall_threshold: 
            self.wall_distance[0] = self.current_distance[0]
            self.get_logger().info("left distance: {}".format(self.wall_distance[0]))
        if dec_right > self.wall_threshold: 
            self.wall_distance[1] = self.current_distance[1]
            self.get_logger().info("right distance: {}".format(self.wall_distance[1]))

        if self.marker_state == 0 and self.dec_upper_threshold > (self.wall_distance[0] - self.current_distance[0]) > self.dec_lower_threshold:
            if abs(self.y - self.current_marker[1]) < 0.4:
                return
            detection_msg = String()
            if dec_left < 0.07:
                detection_msg.data = "FERTILIZER_REQUIRED," + str(self.x) + ',' + str(self.y) + ',' + str(self.tray)
                self.last_msg = "FERTILIZER_REQUIRED,"
            else:
                detection_msg.data = "BAD_HEALTH," + str(self.x) + ',' + str(self.y) + ',' + str(self.tray)
                self.last_msg = "BAD_HEALTH,"
            self.publisher.publish(detection_msg)
            self.direction = 0
            self.count += 1
            self.last_detection_time = self.get_clock().now()
            self.marker_state = 1
            self.current_marker = [self.x, self.y]

        elif self.marker_state == 0 and self.dec_upper_threshold > (self.wall_distance[1] - self.current_distance[1]) > self.dec_lower_threshold:
            if abs(self.y - self.current_marker[1]) < 0.4:
                return
            detection_msg = String()
            if dec_right < 0.07:
                detection_msg.data = "FERTILIZER_REQUIRED," + str(self.x) + ',' + str(self.y) + ',' + str(self.tray)
                self.last_msg = "FERTILIZER_REQUIRED,"
            else:
                detection_msg.data = "BAD_HEALTH," + str(self.x) + ',' + str(self.y) + ',' + str(self.tray)
                self.last_msg = "BAD_HEALTH,"
            self.publisher.publish(detection_msg)
            self.direction = 1
            self.count += 1
            self.last_detection_time = self.get_clock().now()
            self.marker_state = 1
            self.current_marker = [self.x, self.y]

        
        elif self.marker_state == 1 and ((self.dec_upper_threshold > (self.wall_distance[0] - self.current_distance[0]) > self.dec_lower_threshold) or (self.dec_upper_threshold > (self.wall_distance[1] - self.current_distance[1]) > self.dec_lower_threshold)):
            self.get_logger().info(f"x0: {self.current_distance[0]}")
            self.get_logger().info(f"x1: {self.current_distance[1]}")
            self.marker_state = 1

        else:
            self.marker_state = 0


        self.previous_distance = self.current_distance.copy()  


rclpy.init()
node = ShapeDetectorNode()
rclpy.spin(node)
rclpy.shutdown()