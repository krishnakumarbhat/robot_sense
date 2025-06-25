import pyrealsense2 as rs
import numpy as np
import cv2
import time

def d435i_reader(duration_sec=15):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

    pipeline.start(config)
    print(f"Streaming color at 640x480, 15 FPS for {duration_sec} seconds.")

    start_time = time.time()
    frames_data = []

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            timestamp = time.time() - start_time
            frames_data.append((timestamp, color_image))

            cv2.imshow('D435i Color Stream', color_image)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if (time.time() - start_time) > duration_sec:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return frames_data


