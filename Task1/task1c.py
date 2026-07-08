import rclpy
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Quaternion
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from math import acos, sin


class MoveArm(Node):
    def __init__(self):
        super().__init__("arm_controller")
        self.publisher = self.create_publisher(Twist, "/delta_twist_cmds", 10)
        self.timer = self.create_timer(0.1, self.publish_msg)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.Kp = 0.35
        self.Ki = 0.003
        self.pos_tol = 0.15
        self.ori_tol = 0.15
        self.current_target = 0
        self.cum_error_x = 0.0
        self.cum_error_y = 0.0
        self.cum_error_z = 0.0
        self.cum_error_qx = 0.0
        self.cum_error_qy = 0.0
        self.cum_error_qz = 0.0
        self.cum_error_qw = 0.0
        self.current_pos = TransformStamped()
        self.target_pos = [TransformStamped() for _ in range(3)]
        self.target_pos[1].transform.translation = Vector3(x = -0.214, y = -0.532, z = 0.557)
        self.target_pos[1].transform.rotation = Quaternion(x = 0.707, y = 0.028, z = 0.034, w = 0.707)
        self.target_pos[0].transform.translation = Vector3(x = -0.159, y = 0.501, z = 0.415)
        self.target_pos[0].transform.rotation = Quaternion(x = 0.029, y = 0.997, z = 0.045, w = 0.033)
        self.target_pos[2].transform.translation = Vector3(x = -0.806, y = 0.010, z = 0.182)
        self.target_pos[2].transform.rotation = Quaternion(x = -0.684, y = 0.726, z = 0.05, w = 0.008)
    
    def quaternion_inverse(self, q):
        return Quaternion(x = -q.x, y = -q.y, z = -q.z, w = q.w)
    
    def quaternion_multiply(self, q1, q2):
        return Quaternion(
            x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
            y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x,
            z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w,
            w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        )
    
    def publish_msg(self):
        if not self.buffer.can_transform("base_link", "wrist_3_link", Time()):
            return
        
        transform = self.buffer.lookup_transform("base_link", "wrist_3_link", Time())
        self.current_pos.transform = transform.transform
        curr_pos = self.current_pos.transform.translation
        curr_ori = self.current_pos.transform.rotation
        tar_pos = self.target_pos[self.current_target].transform.translation
        tar_ori = self.target_pos[self.current_target].transform.rotation

        error_x = tar_pos.x - curr_pos.x
        error_y = tar_pos.y - curr_pos.y
        error_z = tar_pos.z - curr_pos.z
        error_q = self.quaternion_multiply(tar_ori, self.quaternion_inverse(curr_ori))
        error_qx = error_q.x
        error_qy = error_q.y
        error_qz = error_q.z
        error_qw = error_q.w

        theta = 2 * acos(np.clip(error_qw, -1.0, 1.0))
        sin_theta_by_2 = sin(theta / 2)

        if (error_x**2 + error_y**2 + error_z**2) < self.pos_tol**2 and abs(theta) < self.ori_tol:
            if self.current_target == 2:
                self.get_logger().info("All targets reached. Shutting down.")
                rclpy.shutdown()
            else:
                self.current_target += 1
                self.get_logger().info(f"Reached target {self.current_target}")
                self.publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))
                time.sleep(1)
                return
        
        if (error_x**2 + error_y**2 + error_z**2) < self.pos_tol**2:
            self.get_logger().info(f"Position reached for target {self.current_target}, adjusting orientation.")

        if abs(sin_theta_by_2) < 1e-8:
            error_qx = 0
            error_qy = 0
            error_qz = 0
        else:
            error_qx /= sin_theta_by_2
            error_qy /= sin_theta_by_2
            error_qz /= sin_theta_by_2

        self.cum_error_x += error_x
        self.cum_error_y += error_y
        self.cum_error_z += error_z
        self.cum_error_qx += error_qx
        self.cum_error_qy += error_qy
        self.cum_error_qz += error_qz

        move = Twist()
        move.linear.x = self.Kp * error_x + self.Ki * self.cum_error_x
        move.linear.y = self.Kp * error_y + self.Ki * self.cum_error_y
        move.linear.z = self.Kp * error_z + self.Ki * self.cum_error_z
        move.angular.x = 1.2 * theta * error_qx
        move.angular.y = 1.2 * theta * error_qy
        move.angular.z = 1.2 * theta * error_qz
        self.publisher.publish(move)


rclpy.init()
node = MoveArm()
rclpy.spin(node)
rclpy.shutdown()
