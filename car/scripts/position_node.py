#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Quaternion
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class IMUIntegrationNode(Node):
    def __init__(self):
        super().__init__('imu_integration_node')
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.imu_sub = self.create_subscription(Imu, '/imu_plugin/out', self.imu_callback, qos_profile=self.qos_profile)
        self.pose_pub = self.create_publisher(PoseStamped, '/car_pose', 10)
        self.current_pose = PoseStamped()
        self.previous_time = self.get_clock().now()
        self.velocity_result = np.zeros(3)  # Initialize velocity as [0, 0, 0]
        self.position_result = np.zeros(3)  # Initialize position as [0, 0, 0]
        self.orientation_result = Quaternion() # Initialize orientation as no rotation (identity)

    def imu_callback(self, imu_msg):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.previous_time).nanoseconds * 1e-9
        
        # Integrate linear acceleration to get velocity
        linear_acc = imu_msg.linear_acceleration
        velocity_change = [linear_acc.x * delta_time, linear_acc.y * delta_time, linear_acc.z * delta_time]
        self.velocity_result += velocity_change

        # Integrate velocity to get position
        position_change = [self.velocity_result[0] * delta_time, self.velocity_result[1] * delta_time, self.velocity_result[2] * delta_time]
        self.position_result -= position_change

        # Integrate angular velocity to get orientation change
        angular_velocity = imu_msg.angular_velocity
        orientation_change = Quaternion()
        orientation_change.w = 1.0  # Identity quaternion
        orientation_change.x = round(angular_velocity.x * delta_time,3)
        orientation_change.y = round(angular_velocity.y * delta_time,3)
        orientation_change.z = round(angular_velocity.z * delta_time,3)
        self.orientation_result = self.quaternion_multiply(self.orientation_result, orientation_change)

        # Update the current pose with the estimated position and orientation
        self.current_pose.pose.position.x = round(self.position_result[0],7)
        self.current_pose.pose.position.y = round(self.position_result[1],7)
        self.current_pose.pose.position.z = 1.0
        self.current_pose.pose.orientation = self.orientation_result
        # Publish the updated pose
        self.pose_pub.publish(self.current_pose)
        self.previous_time = current_time
        self.get_logger().info(f'current position {self.current_pose.pose.position.y}')

    def quaternion_multiply(self, q1, q2):
        result = Quaternion()
        result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z
        result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y
        result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x
        result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w
        return result

def main(args=None):
    rclpy.init(args=args)
    imu_integration_node = IMUIntegrationNode()
    rclpy.spin(imu_integration_node)
    imu_integration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()