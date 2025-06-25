import sys
from sense.d435i_reader import d435i_reader
from store.bag_storer import BagStorage
from ros_publisher import RosPublisher

def main():
    camera = d435i_reader()
    if '-bag' in sys.argv:
        storage = BagStorage()
        camera.start()
        for frame in camera:
            storage.save(frame)
        camera.stop()
    else:
        publisher = RosPublisher()
        camera.start()
        for frame in camera:
            publisher.publish(frame)
        camera.stop()

if __name__ == "__main__":
    main()