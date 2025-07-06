#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyrealsense2 as rs
import numpy as np
from sensor_msgs.msg import Image, PointCloud2, Imu
from sensor_msgs.msg import PointField
import std_msgs.msg as std_msgs
from cv_bridge import CvBridge
import keyboard
import time

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        
        # Declare parameters
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        # Setup publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/camera/pointcloud', 10)
        self.imu_pub = self.create_publisher(Imu, '/camera/imu', 10)
        self.trigger_pub = self.create_publisher(std_msgs.Bool, '/camera/trigger', 10)
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.accel)
        self.config.enable_stream(rs.stream.gyro)
        
        # Start pipeline
        self.pipeline.start(self.config)
        
        # Create timer for frame processing
        self.timer = self.create_timer(1.0/fps, self.process_frames)
        
        # Create timer for keyboard checking
        self.key_timer = self.create_timer(0.1, self.check_keyboard)
        
        self.bridge = CvBridge()
        self.pc = rs.pointcloud()
        self.get_logger().info('RealSense node started')
    
    def check_keyboard(self):
        try:
            if keyboard.is_pressed('s'):
                trigger_msg = std_msgs.Bool()
                trigger_msg.data = True
                self.trigger_pub.publish(trigger_msg)
                self.get_logger().info('Trigger activated!')
                time.sleep(0.5)  # Debounce
        except:
            pass
    
    def process_frames(self):
        # Wait for frames
        frames = self.pipeline.wait_for_frames()
        
        # Get timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Process and publish color frames
        color_frame = frames.get_color_frame()
        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            msg.header.stamp = timestamp
            msg.header.frame_id = "camera_color_optical_frame"
            self.rgb_pub.publish(msg)
            
        # Process and publish depth frames
        depth_frame = frames.get_depth_frame()
        if depth_frame:
            depth_image = np.asanyarray(depth_frame.get_data())
            msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="mono16")
            msg.header.stamp = timestamp
            msg.header.frame_id = "camera_depth_optical_frame"
            self.depth_pub.publish(msg)
            
            # Generate and publish pointcloud
            points = self.pc.calculate(depth_frame)
            vertices = np.asanyarray(points.get_vertices())
            
            # Create PointCloud2 message
            pc2_msg = PointCloud2()
            pc2_msg.header.stamp = timestamp
            pc2_msg.header.frame_id = "camera_depth_optical_frame"
            pc2_msg.height = 1
            pc2_msg.width = len(vertices)
            pc2_msg.fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
            ]
            pc2_msg.is_bigendian = False
            pc2_msg.point_step = 12  # 3 * float32 (4 bytes)
            pc2_msg.row_step = pc2_msg.point_step * len(vertices)
            pc2_msg.data = vertices.tobytes()
            pc2_msg.is_dense = True
            
            self.pointcloud_pub.publish(pc2_msg)
        
        # Process IMU data
        accel = frames.first_or_default(rs.stream.accel)
        gyro = frames.first_or_default(rs.stream.gyro)
        if accel and gyro:
            imu_msg = Imu()
            imu_msg.header.stamp = timestamp
            imu_msg.header.frame_id = "camera_imu_optical_frame"
            
            # Set accelerometer data
            imu_msg.linear_acceleration.x = accel.as_motion_frame().get_motion_data().x
            imu_msg.linear_acceleration.y = accel.as_motion_frame().get_motion_data().y
            imu_msg.linear_acceleration.z = accel.as_motion_frame().get_motion_data().z
            
            # Set gyroscope data
            imu_msg.angular_velocity.x = gyro.as_motion_frame().get_motion_data().x
            imu_msg.angular_velocity.y = gyro.as_motion_frame().get_motion_data().y
            imu_msg.angular_velocity.z = gyro.as_motion_frame().get_motion_data().z
            
            self.imu_pub.publish(imu_msg)
    
    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()