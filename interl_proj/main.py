import pyrealsense2 as rs
import numpy as np
import time
import os
import csv

# --- Configuration ---
OUTPUT_DIR = "realsense_output_d435i" # Directory to store all output files
CAPTURE_DURATION_SEC = 10        # Capture data for this many seconds
# Or uncomment below to capture a fixed number of frames instead
# CAPTURE_FRAME_COUNT = 100

# Stream Configuration (Adjust based on D435i capabilities and needs)
DEPTH_WIDTH = 424 # Lower resolution example
DEPTH_HEIGHT = 240
COLOR_WIDTH = 424 # Lower resolution example
COLOR_HEIGHT = 240
FRAME_RATE = 15 # Target framerate for Depth/Color

# --- Important Note on "Geolocation" ---
# The D435i does NOT have GPS.
# This code captures IMU (accelerometer, gyroscope) data.
# This data allows tracking the camera's RELATIVE motion and orientation (pose),
# which is often used in robotics instead of absolute GPS coordinates.
# Downstream processing (like SLAM algorithms) is needed to compute pose from IMU data.

# --- Setup Output Directories and Files ---
pc_folder = os.path.join(OUTPUT_DIR, "point_clouds")
os.makedirs(pc_folder, exist_ok=True)
imu_csv_path = os.path.join(OUTPUT_DIR, "imu_data.csv")
metadata_csv_path = os.path.join(OUTPUT_DIR, "metadata.csv")
print(f"Output will be saved to: {os.path.abspath(OUTPUT_DIR)}")

pipeline = None
imu_file = None
imu_writer = None
metadata_file = None
metadata_writer = None

try:
    # --- Initialize Pipeline ---
    pipeline = rs.pipeline()
    config = rs.config()

    # Check for device
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        raise RuntimeError("No RealSense device detected. Is it plugged in?")
    device = ctx.query_devices()[0] # Get the first device
    print(f"Found device: {device.get_info(rs.camera_info.name)}")
    if "D435I" not in device.get_info(rs.camera_info.name).upper():
         print("Warning: The connected device might not be a D435i. IMU streams may not be available.")


    # --- Enable Streams ---
    print("Enabling streams...")
    # Depth Stream
    config.enable_stream(rs.stream.depth, DEPTH_WIDTH, DEPTH_HEIGHT, rs.format.z16, FRAME_RATE)
    # Color Stream
    config.enable_stream(rs.stream.color, COLOR_WIDTH, COLOR_HEIGHT, rs.format.bgr8, FRAME_RATE)
    # IMU Streams (Check D435i specs/SDK examples for exact supported formats/rates if needed)
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f) # MOTION_XYZ32F is common
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f)  # MOTION_XYZ32F is common
    print("Streams enabled (Depth, Color, Accel, Gyro).")

    # --- Start Pipeline ---
    print("Starting pipeline...")
    profile = pipeline.start(config)
    print("Pipeline started.")

    # Setup alignment object (align depth to color)
    align_to = rs.stream.color
    align = rs.align(align_to)

    # --- Setup CSV Writers ---
    # IMU Data CSV
    imu_file = open(imu_csv_path, 'w', newline='')
    imu_writer = csv.writer(imu_file)
    imu_writer.writerow(['timestamp_rs', 'timestamp_sys', 'stream_type', 'x', 'y', 'z'])
    print(f"IMU data will be saved to: {imu_csv_path}")

    # Metadata CSV (linking point clouds to time)
    metadata_file = open(metadata_csv_path, 'w', newline='')
    metadata_writer = csv.writer(metadata_file)
    metadata_writer.writerow(['frame_number', 'timestamp_rs_depth', 'timestamp_sys', 'point_cloud_filename'])
    print(f"Metadata will be saved to: {metadata_csv_path}")
    print(f"Point clouds will be saved in: {pc_folder}")


    # --- Allow camera to stabilize ---
    print("Waiting for auto-exposure/white-balance and IMU to stabilize...")
    for _ in range(int(FRAME_RATE * 1.5)): # Wait about 1.5 seconds worth of frames
       pipeline.wait_for_frames()
    print("Stabilization complete.")

    # --- Capture Loop ---
    point_cloud_count = 0
    last_color_frame = None # Keep track of the latest color frame for texturing
    start_time = time.time()
    last_print_time = start_time

    print(f"Starting capture loop...")
    if 'CAPTURE_DURATION_SEC' in globals():
        print(f"Will capture for approximately {CAPTURE_DURATION_SEC} seconds.")
    elif 'CAPTURE_FRAME_COUNT' in globals():
         print(f"Will capture {CAPTURE_FRAME_COUNT} point cloud frames.")

    while True:
        # Check termination condition
        current_time = time.time()
        if 'CAPTURE_DURATION_SEC' in globals():
            if (current_time - start_time) >= CAPTURE_DURATION_SEC:
                print("\nCapture duration reached.")
                break
        elif 'CAPTURE_FRAME_COUNT' in globals():
             if point_cloud_count >= CAPTURE_FRAME_COUNT:
                 print(f"\nCaptured {point_cloud_count} frames.")
                 break
        else:
             # Default: Run for a short time if no condition set
             if (current_time - start_time) >= 5: # Default 5 seconds
                 print("\nDefault capture time (5s) reached.")
                 break

        # --- Wait for Frameset ---
        try:
            # Wait for a coherent set of frames (depth, color, accel, gyro)
            # Timeout prevents blocking indefinitely if streams stop
            frames = pipeline.wait_for_frames(timeout_ms=2000) # 2 second timeout
            if not frames:
                print("Warning: No frames received within timeout.")
                continue
        except RuntimeError as e:
            print(f"Runtime error waiting for frames: {e}. Attempting to continue...")
            time.sleep(0.1) # Brief pause before retrying
            continue

        # Get system timestamp as close as possible to frame arrival
        current_sys_time = time.time()

        # --- Process All Frames in Frameset ---
        depth_frame_for_pc = None # Track if we got a depth frame in this set
        color_frame_for_pc = None # Track the color frame associated via alignment

        for frame in frames:
            frame_profile = frame.get_profile()
            stream_type = frame_profile.stream_type()
            timestamp_rs = frame.get_timestamp() # Use device timestamp

            # --- Process IMU Frame ---
            if frame.is_motion_frame():
                motion_data = frame.as_motion_frame().get_motion_data()
                if stream_type == rs.stream.accel:
                    data_type = 'accel'
                    imu_writer.writerow([timestamp_rs, current_sys_time, data_type,
                                         motion_data.x, motion_data.y, motion_data.z])
                elif stream_type == rs.stream.gyro:
                    data_type = 'gyro'
                    imu_writer.writerow([timestamp_rs, current_sys_time, data_type,
                                         motion_data.x, motion_data.y, motion_data.z])

            # --- Process Video Frames (Depth/Color) ---
            elif frame.is_video_frame():
                # Store the latest color frame seen in this frameset
                # (Alignment will use this later)
                if stream_type == rs.stream.color:
                     last_color_frame = frame
                # Identify the depth frame in this set
                elif stream_type == rs.stream.depth:
                     depth_frame_for_pc = frame


        # --- Perform Alignment and Generate Point Cloud (if depth frame arrived) ---
        if depth_frame_for_pc and last_color_frame:
             # Align the frames from *this specific frameset*
             # This ensures temporal consistency for the point cloud
             try:
                 aligned_frames = align.process(frames)
                 aligned_depth_frame = aligned_frames.get_depth_frame()
                 aligned_color_frame = aligned_frames.get_color_frame() # Use color from aligned set

                 if aligned_depth_frame and aligned_color_frame:
                    # --- Generate Point Cloud ---
                    pc = rs.pointcloud()
                    pc.map_to(aligned_color_frame) # Map texture coordinates from color frame
                    points = pc.calculate(aligned_depth_frame) # Calculate 3D vertices + UV map

                    # --- Save Point Cloud ---
                    ply_filename = f"frame_{point_cloud_count:05d}.ply" # Use 5 digits padding
                    ply_filepath = os.path.join(pc_folder, ply_filename)
                    points.export_to_ply(ply_filepath, aligned_color_frame) # Save with color

                    # --- Record Metadata ---
                    depth_timestamp_rs = aligned_depth_frame.get_timestamp()
                    metadata_writer.writerow([point_cloud_count, depth_timestamp_rs, current_sys_time, ply_filename])

                    point_cloud_count += 1

                    # Print progress periodically
                    if current_time - last_print_time >= 1.0: # Print status roughly every second
                         print(f"\rCaptured Point Cloud Frame: {point_cloud_count} | Time: {current_time - start_time:.1f}s", end="")
                         last_print_time = current_time

                 else:
                    print("Warning: Alignment failed for current frameset.")

             except Exception as e:
                 print(f"\nError during alignment or point cloud processing: {e}")
                 # Decide whether to continue or break based on error severity


    print(f"\nCapture complete. Saved {point_cloud_count} point clouds.")

except Exception as e:
    print(f"\nAn error occurred outside the main loop: {e}")
    import traceback
    traceback.print_exc()

finally:
    # --- Cleanup ---
    if imu_file:
        imu_file.close()
        print(f"IMU data saved to: {imu_csv_path}")
    if metadata_file:
        metadata_file.close()
        print(f"Metadata saved to: {metadata_csv_path}")
    if pipeline:
        print("Stopping pipeline...")
        pipeline.stop()
        print("Pipeline stopped.")
    else:
        print("Pipeline was not initialized.")

print(f"\nOutput data is located in: {os.path.abspath(OUTPUT_DIR)}")
print("Data structure:")
print(f" - {pc_folder}/ : Contains frame_xxxxx.ply files (3D point clouds)")
print(f" - {imu_csv_path} : Contains raw accelerometer and gyroscope readings")
print(f" - {metadata_csv_path} : Links point cloud frames to filenames and timestamps")