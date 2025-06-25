
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import csv
import time

# Stream parameter configuration
config = rs.config()
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f)
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f)

# Timestamp conversion utilities
def rs_timestamp_to_ns(ts):
    return int(ts * 1000)  # Convert ms to ns

def system_time_to_ns():
    return time.time_ns()


def process_imu_frame(frame, writer):
    motion_data = frame.as_motion_frame().get_motion_data()
    stream_type = frame.get_profile().stream_type()
    return {
        'timestamp': frame.get_timestamp(),
        'type': 'accel' if stream_type == rs.stream.accel else 'gyro',
        'x': motion_data.x,
        'y': motion_data.y,
        'z': motion_data.z
    }


class IMULogger:
    def __init__(self, filename):
        self.file = open(filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['system_ns', 'sensor_ns', 'type', 'x', 'y', 'z'])
    
    def write_entry(self, data):
        sensor_ns = rs_timestamp_to_ns(data['timestamp'])
        self.writer.writerow([
            system_time_to_ns(),
            sensor_ns,
            data['type'],
            data['x'],
            data['y'],
            data['z']
        ])

# Sensor Data Flow:
# IMU Raw → Body Frame → Depth Sensor Frame → World Frame


class KalmanFilter:
    def __init__(self):
        # State vector: [x, y, z, vx, vy, vz, ax, ay, az]
        self.state = np.zeros(9)
        self.covariance = np.eye(9)
    
    def predict(self, gyro, dt):
        # Implement quaternion-based prediction step
        pass
    
    def update(self, depth, accel):
        # Measurement update from depth and acceleration
        pass


from threading import Thread
from queue import Queue

class DataProcessor(Thread):
    def __init__(self, input_queue):
        super().__init__()
        self.queue = input_queue
    
    def run(self):
        while True:
            data = self.queue.get()
            if data is None:
                break
            # Process data package

def frame_to_array(frame):
    return np.asanyarray(frame.get_data())


try:
    frames = pipeline.wait_for_frames(timeout_ms=2000)
except rs.error as e:
    print(f"RealSense error: {e.get_failed_function()}: {e.get_failed_args()}")
    reconnect_sensor()


class HealthMonitor:
    def __init__(self):
        self.frame_counts = {
            'depth': 0,
            'color': 0,
            'accel': 0,
            'gyro': 0
        }
    
    def update_counts(self, frames):
        for frame in frames:
            stream = frame.get_profile().stream_name()
            self.frame_counts[stream] += 1


import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class IMUVisualizer:
    def __init__(self):
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)
        self.accel_lines = self.ax1.plot([], [], label=['X', 'Y', 'Z'])
        self.gyro_lines = self.ax2.plot([], [], label=['X', 'Y', 'Z'])
    
    def update_plot(self, frame_data):
        # Update plot data
        pass


def configure_power_modes(device):
    depth_sensor = device.first_depth_sensor()
    depth_sensor.set_option(rs.option.power_line_frequency, 60)
    depth_sensor.set_option(rs.option.enable_auto_exposure, 1)
