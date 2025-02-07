#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from datetime import datetime

class VideoRecorder:
    def __init__(self):
        rospy.init_node('video_recorder', anonymous=True)
        
        # Subscribe to the RealSense camera image topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.bridge = CvBridge()
        
        # Get the current date and time for the video filename
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.video_filename = f"realsense_video_{timestamp}.avi"
        
        # Video writer setup
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for AVI
        self.out = None
        self.frame_width = None
        self.frame_height = None
        rospy.loginfo(f"Recording video to {self.video_filename}")
        
    def image_callback(self, data):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            
            # Initialize VideoWriter once we get the first frame
            if self.out is None:
                self.frame_height, self.frame_width = cv_image.shape[:2]
                self.out = cv2.VideoWriter(self.video_filename, self.fourcc, 30.0, (self.frame_width, self.frame_height))
            
            # Write the frame to the video
            self.out.write(cv_image)
            
            # Optional: Show the video frame (comment out if not needed)
            # cv2.imshow('Recording Video', cv_image)
            cv2.waitKey(1)  # Needed to render the OpenCV window
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def shutdown(self):
        rospy.loginfo("Shutting down video recorder...")
        if self.out:
            self.out.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        recorder = VideoRecorder()
        rospy.on_shutdown(recorder.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
