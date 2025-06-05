#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
class Kalma_filter(Node):
    def __init__(self):
        super().__init__("kalma_filter")
        self.odom_sub_ = self.create_subscription(Odometry,"my_robot/odom_noisy",self.odomCallback,10)
        self.imu_sub_ = self.create_subscription(Imu,"imu/out" ,self.imuCallback,10)
        self.odom_pub_ = self.create_publisher(Odometry,"my_robot/odom_kalma",10)

        self.mean_ = 0.0
        self.variance_ = 1000.0
        self.imu_angular_z = 0.0
        self.is_first_odom  =True
        self.last_angular_z = 0.0


        self.motion_ = 0.0
        self.kalman_odom = Odometry()
        self.motion_variance_ = 4.0
        self.measure_variance_  = 0.5

    def measureUpdate(self):
        self.mean_ = (self.measure_variance_ * self.mean_ + self.variance_ * self.imu_angular_z) / (self.variance_ + self.measure_variance_)
        self.variance_ = (self.variance_ * self.measure_variance_) / (self.variance_ + self.measure_variance_)
    def statePrediction(self):
        self.mean_ = self.mean_ + self.motion_
        self.variance_ = self.variance_ + self.motion_variance_

    def imuCallback(self, imu):
        self.imu_angular_z = imu.angular_velocity.z
    def odomCallback(self, odom):
        self.kalman_odom = odom
        if self.is_first_odom:
            self.mean = odom.twist.twist.angular.z
            self.last_angular_z = odom.twist.twist.angular.z
            self.is_first_odom = False
            return
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z
        self.measureUpdate()
        self.statePrediction()
        self.kalman_odom.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom)
def main():
    rclpy.init()
    node = Kalma_filter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()
