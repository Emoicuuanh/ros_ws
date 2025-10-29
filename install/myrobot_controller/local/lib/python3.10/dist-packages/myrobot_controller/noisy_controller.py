#!/usr/bin//env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from sensor_msgs.msg import JointState
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

class NoisyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")
        self.declare_parameter("wheel_radius",0.1)
        self.declare_parameter("wheel_serapation",0.4)

        self.left_wheel_pre_pos = 0.0
        self.right_wheel_pre_pos = 0.0
        self.pre_time_ = self.get_clock().now()

        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0
        
        self.wheel_radius_ =self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_serapation_ =self.get_parameter("wheel_serapation").get_parameter_value().double_value

        self.joint_sub_ =self.create_subscription(JointState,"joint_states",self.jointCallback,10)
        self.odom_pub_ = self.create_publisher(Odometry,"my_robot/odom_noisy",10)

        self.br_ =TransformBroadcaster(self)
        self.br_stamped = TransformStamped()


        self.odom_msg_ =Odometry()
        self.odom_msg_.header.frame_id="odom"
        self.odom_msg_.child_frame_id= "base_footprint_noisy"
        self.odom_msg_.pose.pose.orientation.x =0.0
        self.odom_msg_.pose.pose.orientation.y =0.0
        self.odom_msg_.pose.pose.orientation.z =0.0
        self.odom_msg_.pose.pose.orientation.w =1.0
        
    def jointCallback(self, msg):
        wheel_left_encoder = msg.position[1] + np.random.normal(0,0.0005)
        wheel_right_encoder = msg.position[0] + np.random.normal(0,0.0005)
        dp_left = wheel_left_encoder - self.left_wheel_pre_pos
        dp_right = wheel_right_encoder - self.right_wheel_pre_pos

        dt = Time.from_msg(msg.header.stamp) - self.pre_time_

        fi_left = dp_left/ (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right/ (dt.nanoseconds / S_TO_NS)

        linear = (self.wheel_radius_ * fi_left + self.wheel_radius_ * fi_right)/2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left)/self.wheel_serapation_

        d_s = (self.wheel_radius_ * dp_left + self.wheel_radius_ * dp_right)/2
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left)/self.wheel_serapation_

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)
        q =quaternion_from_euler(0 , 0 ,self.theta_)

        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        self.br_stamped.header.frame_id="odom"
        self.br_stamped.child_frame_id = "base_footprint_noisy"
        self.br_stamped.transform.rotation.x =q[0]
        self.br_stamped.transform.rotation.y =q[1]
        self.br_stamped.transform.rotation.z =q[2]
        self.br_stamped.transform.rotation.w =q[3]
        self.br_stamped.header.stamp = self.get_clock().now().to_msg()
        self.br_stamped.transform.translation.x = self.x_
        self.br_stamped.transform.translation.y = self.y_

        self.odom_pub_.publish(self.odom_msg_)
        self.br_.sendTransform(self.br_stamped)

        self.left_wheel_pre_pos = msg.position[1]
        self.right_wheel_pre_pos = msg.position[0]
        self.pre_time_ = Time.from_msg(msg.header.stamp)
def main():
    rclpy.init()
    noisy_controller= NoisyController()
    rclpy.spin(noisy_controller)
    noisy_controller.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()