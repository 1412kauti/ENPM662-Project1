#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Quaternion
# import tf2_ros
# from tf2_ros import TransformBroadcaster
from math import *
import numpy as np

class MoveObject(Node):
    def __init__(self):
        super().__init__('move_object_node')
        self.vel_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.pos_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.viz_error_pub = self.create_publisher(Float64, '/error_viz',10)
        self.viz_speed_pub = self.create_publisher(Float64, '/control_viz',10)
        self.sub = self.create_subscription(PoseStamped, '/car_pose', self.pose_callback, 10)
        # self.broadcaster = TransformBroadcaster(self)
        self.target_x = 0.0
        self.target_y = 10.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.kp_vel = 0.5  # Proportional gain
        self.kp_steer = 0.5
        self.max_speed = 20.0  # Maximum linear speed
        self.max_steer_angle = 0.6
        # self.target_angle = self.angle_radians(self.target_x,self.target_y,self.origin_x,self.origin_y)
        # self.start_orientation = Quaternion()
        # self.start_orientation.x = 0.0
        # self.start_orientation.y = 0.0
        # self.start_orientation.z = 0.0
        # self.start_orientation.w = 1.0
        
        # self.target_orientation = Quaternion()
        # self.target_orientation.x = cos(self.target_angle/2)
        # self.target_orientation.y = sin(self.target_angle/2)
        # self.target_orientation.z = 0.0
        # self.target_orientation.w = 1.0

    # def angle_radians(self,x1,y1,x2,y2):
    #     d_th_x = x2 - x1
    #     d_th_y = y2 - y1
    #     angle = atan2(d_th_y,d_th_x)
    #     return angle
    


    # def angle_bn_quat(self,x1,y1,z1,w1,x2,y2,z2,w2):
    #     q1 = np.array([x1,y1,z1,w1])
    #     q2 = np.array([x2,y2,z2,w2])
    #     dot_prd = np.dot(q1,q2)
    #     angle = 2 * np.arccos(np.abs(dot_prd))
    #     return angle

    def pose_callback(self, msg):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        # self.current_orientation = msg.pose.orientation

        

        # error_x = self.target_x - self.current_x
        error_y = self.target_y - self.current_y
        # error_th = self.angle_bn_quat(self.target_orientation.x,self.target_orientation.y,self.target_orientation.z,self.target_orientation.w,self.current_orientation.x,self.current_orientation.y,self.current_orientation.z,self.current_orientation.w)

        linear_speed = self.kp_vel * error_y
        # steering = self.kp_steer * error_th

        
        
        if linear_speed >= self.max_speed:
            linear_speed = self.max_speed
        
        if linear_speed <= -self.max_speed:
            linear_speed = -self.max_speed

        # if steering >= self.max_steer_angle:
        #     steering = self.max_steer_angle
        
        # if steering <= -self.max_steer_angle:
        #     steering = -self.max_steer_angle  

        wheel_velocities = Float64MultiArray()
        # steering_positions = Float64MultiArray()
        wheel_velocities.data = [-linear_speed,-linear_speed,-linear_speed,-linear_speed]
        # steering_positions.data = [steering,steering]
        self.get_logger().info(f'error {error_y}')
        self.vel_pub.publish(wheel_velocities)
        # self.pos_pub.publish(steering_positions)

        viz_error = Float64()
        viz_speed = Float64()
        viz_error.data = error_y
        viz_speed.data = linear_speed
        self.viz_error_pub.publish(viz_error)
        self.viz_speed_pub.publish(viz_speed)


        if error_y < 0.9:
            linear_speed = 0.0
            wheel_velocities.data = [-linear_speed,-linear_speed,-linear_speed,-linear_speed]
            self.vel_pub.publish(wheel_velocities)
            self.get_logger().info('Target distance reached. Stopping the node.')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    move_object = MoveObject()
    rclpy.spin(move_object)
    move_object.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()