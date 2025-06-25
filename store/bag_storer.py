import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BagStorer:
    def __init__(self, topic_name='/camera/color/image_raw'):
        rospy.init_node('bag_storer', anonymous=True)
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(topic_name, Image, queue_size=10)

    def store_frames(self, frames_data):
        rate = rospy.Rate(15)  # 15 Hz to match frame rate
        for timestamp, frame in frames_data:
            if rospy.is_shutdown():
                break
            ros_image = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            ros_image.header.stamp = rospy.Time.now()
            self.pub.publish(ros_image)
            rate.sleep()

if __name__ == "__main__":
    # Example usage:
    # from d435i_reader import d435i_reader
    # frames = d435i_reader()
    # bag_storer = BagStorer()
    # bag_storer.store_frames(frames)
    pass
