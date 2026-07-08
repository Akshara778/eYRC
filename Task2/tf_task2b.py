import rclpy
import numpy as np
import time
from rclpy.node import Node
from linkattacher_msgs.srv import AttachLink, DetachLink
from geometry_msgs.msg import Twist, TransformStamped, Vector3, Quaternion
from std_msgs.msg import Float64MultiArray, Int8
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from rclpy.time import Time
from math import acos, sin

class PickAndDropNode(Node):
    def __init__(self):
        super().__init__("arm_controller")
        self.publisher = self.create_publisher(Twist, "/delta_twist_cmds", 10)
        self.subscriber = self.create_subscription(TFMessage, "/tf", self.set_target, 10)
        self.base_publisher = self.create_publisher(Float64MultiArray, "/delta_joint_cmds", 10)
        self.attach = self.create_client(AttachLink, "/attach_link")
        self.detach = self.create_client(DetachLink, "/detach_link")
        self.flag = self.create_subscription(Int8, "/flag", self.check_start, 10)
        self.timer = self.create_timer(0.05, self.publish_msg)
        self.start = 0
        self.Kp = 2.75
        self.Ki = 0.016
        self.pos_tol = 0.05
        self.ori_tol = 0.1
        self.cum_error_x = 0.0
        self.cum_error_y = 0.0
        self.cum_error_z = 0.0
        self.cum_error_qx = 0.0
        self.cum_error_qy = 0.0
        self.cum_error_qz = 0.0
        self.cum_error_qw = 0.0
        self.current_target = 2
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.current_pos = TransformStamped()
        self.target_pos = [TransformStamped() for _ in range(8)]
        self.target_pos[3].transform.translation = Vector3(x = -0.806, y = 0.010, z = 0.182)
        self.target_pos[5].transform = self.target_pos[3].transform
        self.target_pos[7].transform = self.target_pos[3].transform
    

    def check_start(self, msg : Int8):
        self.start = msg.data

    def set_target(self, msg : TFMessage):
        for transform in msg.transforms:
            if transform.child_frame_id == "4558_fertiliser_can" and self.target_pos[0].header.frame_id == "":
                self.target_pos[0] = transform
            elif transform.child_frame_id == "obj_6" and self.target_pos[1].header.frame_id == "":
                if self.start == 1:
                    self.target_pos[1] = transform
                    self.target_pos[1].transform.translation.z += 0.15
                    self.target_pos[3].transform.rotation = self.target_pos[1].transform.rotation
            elif transform.child_frame_id == "4558_bad_fruit_1" and self.target_pos[2].header.frame_id == "":
                self.target_pos[2] = transform
                self.target_pos[2].transform.translation.z += 0.05
            elif transform.child_frame_id == "4558_bad_fruit_2" and self.target_pos[4].header.frame_id == "":
                self.target_pos[4] = transform
                self.target_pos[4].transform.translation.z += 0.05
            elif transform.child_frame_id == "4558_bad_fruit_3" and self.target_pos[6].header.frame_id == "":
                self.target_pos[6] = transform
                self.target_pos[6].transform.translation.z += 0.05

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
        if self.start == 0:
            return
        if not self.buffer.can_transform("base_link", "wrist_3_link", Time()):
            return
        
        transform = self.buffer.lookup_transform("base_link", "wrist_3_link", Time())
        self.get_logger().info("current target: {}".format(self.current_target))
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

            self.cum_error_x = 0.0
            self.cum_error_y = 0.0
            self.cum_error_z = 0.0
            self.cum_error_qx = 0.0
            self.cum_error_qy = 0.0
            self.cum_error_qz = 0.0
            self.cum_error_qw = 0.0

            if self.current_target == 8:
                self.get_logger().info("All targets reached. Shutting down.")
                rclpy.shutdown()
            else:
                if self.current_target % 2 == 0:
                    request = AttachLink.Request()
                    request.link1_name = "body"
                    request.model2_name = "ur5"
                    request.link2_name = "wrist_3_link"
                    if self.current_target == 0:
                        request.model1_name = "fertiliser_can"
                    else:
                        request.model1_name = "bad_fruit"
                        self.attach.call_async(request)
                        self.timer.cancel()
                        for _ in range(40):
                            self.base_publisher.publish(Float64MultiArray(data = [0.0, 4, 3.5, 0.0, 0.0, 0.0]))
                            time.sleep(0.01)
                        self.timer.reset()
                        self.current_target += 1
                        return
                    self.attach.call_async(request)
                else:
                    request = DetachLink.Request()
                    request.link1_name = "body"
                    request.model2_name = "ur5"
                    request.link2_name = "wrist_3_link"
                    if self.current_target == 1:
                        request.model1_name = "fertiliser_can"
                    else:
                        request.model1_name = "bad_fruit"
                    self.detach.call_async(request)

                self.current_target += 1
                self.get_logger().info(f"Reached target {self.current_target}")
                #self.publisher.publish(Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0)))

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
        move.angular.x = 7.8 * theta * error_qx
        move.angular.y = 7.8 * theta * error_qy
        move.angular.z = 7.8 * theta * error_qz
        self.publisher.publish(move)


rclpy.init()
node = PickAndDropNode()
rclpy.spin(node)
rclpy.shutdown()