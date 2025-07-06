#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rcl_interfaces.msg import ParameterDescriptor
import os
import datetime
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions
from std_msgs.msg import Bool
import threading

class RecorderNode(Node):
    def __init__(self):
        super().__init__('recorder_node')
        
        # Declare parameters
        self.declare_parameter('topics_to_record', 
                              ['/camera/color/image_raw', 
                               '/camera/depth/image_rect_raw',
                               '/camera/pointcloud',
                               '/camera/imu'])
        self.declare_parameter('bag_file_prefix', 'realsense_data')
        
        # Initialize recording state
        self.is_recording = False
        self.current_writer = None
        self.writer_lock = threading.Lock()
        
        # Subscribe to trigger topic
        self.trigger_sub = self.create_subscription(
            Bool,
            '/camera/trigger',
            self.trigger_callback,
            10)
        
        # Create subscriptions for all topics
        self.subscriptions = []
        for topic in self.get_parameter('topics_to_record').value:
            self.create_subscription_for_topic(topic)
            
        self.get_logger().info('Recorder node started')

    def create_new_bag(self):
        # Create bag file with timestamp
        timestamp = datetime.datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
        bag_name = f"{self.get_parameter('bag_file_prefix').value}_{timestamp}"
        bag_dir = os.path.expanduser('~') + '/realsense_bags/' + bag_name
        
        # Ensure directory exists
        os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
        
        # Setup rosbag writer
        writer = SequentialWriter()
        storage_options = StorageOptions(uri=bag_dir, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr')
        writer.open(storage_options, converter_options)
        
        self.get_logger().info(f'Created new bag: {bag_dir}')
        return writer

    def trigger_callback(self, msg):
        if msg.data:  # Trigger activated
            with self.writer_lock:
                if not self.is_recording:
                    self.current_writer = self.create_new_bag()
                    self.is_recording = True
                    self.get_logger().info('Started recording')
                else:
                    if self.current_writer:
                        self.current_writer = None
                        self.is_recording = False
                        self.get_logger().info('Stopped recording')

    def create_subscription_for_topic(self, topic_name):
        # Determine message type (simplified - in real code, you'd use introspection)
        # This is just a placeholder - you'd need to properly determine the message type
        if 'image' in topic_name:
            from sensor_msgs.msg import Image
            msg_type = Image
        elif 'pointcloud' in topic_name:
            from sensor_msgs.msg import PointCloud2
            msg_type = PointCloud2
        elif 'imu' in topic_name:
            from sensor_msgs.msg import Imu
            msg_type = Imu
        else:
            self.get_logger().warn(f'Unknown message type for topic: {topic_name}')
            return
        
        # Create the subscription
        sub = self.create_subscription(
            msg_type,
            topic_name,
            lambda msg, topic=topic_name, type=msg_type: self.topic_callback(msg, topic, type),
            10)
        self.subscriptions.append(sub)
        self.get_logger().info(f'Subscribed to: {topic_name}')
    
    def topic_callback(self, msg, topic_name, msg_type):
        # Only write to bag if recording is active
        with self.writer_lock:
            if self.is_recording and self.current_writer:
                # Write message to bag file
                self.current_writer.write(
                    topic_name,
                    serialize_message(msg),
                    self.get_clock().now().nanoseconds)

def main(args=None):
    rclpy.init(args=args)
    node = RecorderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()