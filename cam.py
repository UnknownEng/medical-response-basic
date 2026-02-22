#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from datetime import datetime

class ArucoChunkRecorderNode:
    def __init__(self, chunk_duration=60):
        rospy.init_node('aruco_chunk_recorder_node', anonymous=True)

        # ROS image publisher
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()

        # Initialize PiCamera
        self.camera = PiCamera()
        self.camera.resolution = (640, 480)
        self.camera.framerate = 30
        self.raw_capture = PiRGBArray(self.camera, size=(640, 480))
        time.sleep(2)

        # ArUco detection setup
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # Video chunking setup
        self.chunk_duration = chunk_duration  # seconds
        self.start_time = time.time()
        self.video_writer = self.create_new_video_writer()

        rospy.loginfo("Aruco Chunk Recorder Node Initialized")
        self.run()

    def create_new_video_writer(self):
        filename = datetime.now().strftime("aruco_video_%Y%m%d_%H%M%S.avi")
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        rospy.loginfo(f"Recording new video chunk: {filename}")
        return cv2.VideoWriter(filename, fourcc, 30.0, (640, 480))

    def run(self):
        for frame in self.camera.capture_continuous(self.raw_capture, format="bgr", use_video_port=True):
            image = frame.array
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # ArUco detection
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            if ids is not None:
                # Check for large enough marker
                detected = any(cv2.contourArea(corner) > 1000 for corner in corners)
                if detected:
                    rospy.loginfo("Aruco marker detected!")
                    rospy.loginfo("Landing initiated!")

            # Publish ROS image
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))

            # Write video chunk
            self.video_writer.write(image)

            # Check if chunk duration exceeded
            if time.time() - self.start_time >= self.chunk_duration:
                self.video_writer.release()
                rospy.loginfo("Saved video chunk")
                self.video_writer = self.create_new_video_writer()
                self.start_time = time.time()

            # Clear the stream for the next frame
            self.raw_capture.truncate(0)

            if rospy.is_shutdown():
                break

        self.cleanup()

    def cleanup(self):
        self.video_writer.release()
        self.camera.close()
        rospy.loginfo("Video saved successfully. Node shutting down.")

if __name__ == "__main__":
    try:
        ArucoChunkRecorderNode(chunk_duration=60)  # 60 sec chunks
    except rospy.ROSInterruptException:
        pass
